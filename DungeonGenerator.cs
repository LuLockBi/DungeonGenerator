using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using UnityEngine.SceneManagement;

public class DungeonGenerator : MonoBehaviour
{
    [SerializeField] private int numberOfFloors = 3;
    [SerializeField] private int numberOfRoomsPerFloor = 10;
    [SerializeField] private int maxStaircases = 2;
    [SerializeField] private Vector2 roomSizeMin = new Vector2(4, 4);
    [SerializeField] private Vector2 roomSizeMax = new Vector2(8, 8);
    [SerializeField] private float bufferDistance = 1f;
    [SerializeField] private float floorHeight = 2f;
    public float gridSize = 100f;
    [SerializeField] private GameObject wallFloorPrefab;
    [SerializeField] private GameObject floorPrefab;
    [SerializeField] private GameObject cornerPrefab;
    [SerializeField] private GameObject doorWallPrefab;
    [SerializeField] private GameObject staircasePrefab;
    [SerializeField] private float extraEdgeProbability = 0.15f;
    [SerializeField] private GameObject corridorPrefab;
    [SerializeField] private GameObject corridorNearRoomPrefab;
    [SerializeField] private GameObject corridorJointPrefab;
    [SerializeField] private GameObject crossCorridorPrefab;
    [SerializeField] private GameObject tCorridorPrefab;
    [SerializeField] private GameObject multiRoadTCorridorPrefab;
    [SerializeField] private GameObject multiRoadCorridorJointPrefab;
    [SerializeField] private float clusterLoadRadius = 10f;

    private GameObject player;
    private Dictionary<int, List<List<Vector2Int>>> floorClusters = new Dictionary<int, List<List<Vector2Int>>>();
    private Dictionary<int, Dictionary<List<Vector2Int>, GameObject>> clusterObjects = new Dictionary<int, Dictionary<List<Vector2Int>, GameObject>>();
    private List<List<Room>> floors = new List<List<Room>>();
    private List<List<Triangle>> triangulations = new List<List<Triangle>>();
    private List<List<(Vector2, Vector2)>> corridors = new List<List<(Vector2, Vector2)>>();
    private List<List<List<Vector2Int>>> corridorPaths = new List<List<List<Vector2Int>>>();
    private List<List<Staircase>> staircases = new List<List<Staircase>>();
    private GridCell[,,] grid;
    private int gridResolution;
    private List<GameObject> clusters = new List<GameObject>();

    void Awake()
    {
        clusters = new List<GameObject>();
    }

    public void Regenerate()
    {
        ClearDungeon();
        InitializeGrid();
        GenerateDungeon();
        FindAllClusters();
        SerializeDungeonToJSON();
        LoadInitialClusters();
    }

    private void ClearDungeon()
    {
        foreach (GameObject obj in GameObject.FindGameObjectsWithTag("Room")) Destroy(obj);
        foreach (GameObject obj in GameObject.FindGameObjectsWithTag("Corridor")) Destroy(obj);
        foreach (GameObject obj in GameObject.FindGameObjectsWithTag("Staircase")) Destroy(obj);

        floors.Clear();
        triangulations.Clear();
        corridors.Clear();
        corridorPaths.Clear();
        staircases.Clear();
        floorClusters.Clear();
        clusterObjects.Clear();

        if (grid != null)
        {
            for (int f = 0; f < numberOfFloors; f++)
                for (int x = 0; x < gridResolution; x++)
                    for (int y = 0; y < gridResolution; y++)
                    {
                        grid[x, y, f].type = CellType.Empty;
                        grid[x, y, f].roomPrefabType = RoomPrefabType.None;
                    }
        }
    }

    public enum CellType { Empty, Room, Corridor, CorridorNearRoom, Staircase }
    public enum RoomPrefabType { None, WallFloor, Floor, Corner, DoorWall, StaircaseFloor }

    [System.Serializable]
    private class Room
    {
        public Vector2 center;
        public Vector2 size;
        public Bounds bounds;
        public Bounds tightBounds;
        public GridCell[,] roomGrid;
        public int floorIndex;

        public Room(Vector2 center, Vector2 size, float buffer, int floor)
        {
            this.center = center;
            this.size = size;
            this.bounds = new Bounds(center, size + Vector2.one * buffer * 2f);
            this.tightBounds = new Bounds(center, size);
            this.floorIndex = floor;
            int width = Mathf.CeilToInt(size.x);
            int height = Mathf.CeilToInt(size.y);
            roomGrid = new GridCell[width, height];
            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                    roomGrid[x, y] = new GridCell();
        }
    }

    [System.Serializable]
    private class Staircase
    {
        public Vector2 position;
        public Vector2Int gridPosition;
        public int fromFloor;
        public int toFloor;
        public Vector2Int direction;
        public Room attachedRoom;
        public Vector2Int exitGridPosition;
    }

    [System.Serializable]
    public class GridCell
    {
        public CellType type = CellType.Empty;
        public RoomPrefabType roomPrefabType = RoomPrefabType.None;
    }

    private class Triangle
    {
        public Vector2 p1, p2, p3;
        public Triangle(Vector2 p1, Vector2 p2, Vector2 p3) { this.p1 = p1; this.p2 = p2; this.p3 = p3; }

        public bool IsPointInCircumcircle(Vector2 point)
        {
            float ax = p1.x, ay = p1.y;
            float bx = p2.x, by = p2.y;
            float cx = p3.x, cy = p3.y;
            float dx = point.x, dy = point.y;
            float det = (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) * 2;
            if (det == 0) return false;
            float centerX = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / det;
            float centerY = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / det;
            float radius = Mathf.Sqrt((ax - centerX) * (ax - centerX) + (ay - centerY) * (ay - centerY));
            float dist = Mathf.Sqrt((dx - centerX) * (dx - centerX) + (dy - centerY) * (dy - centerY));
            return dist <= radius;
        }
    }

    private class Node
    {
        public Vector2Int position;
        public int floor;
        public float gCost;
        public float hCost;
        public Node parent;
        public float FCost => gCost + hCost;

        public Node(Vector2Int pos, int floor, float g, float h, Node p)
        {
            position = pos;
            this.floor = floor;
            gCost = g;
            hCost = h;
            parent = p;
        }
    }

    [System.Serializable]
    private class DungeonData { public List<FloorData> floors; }
    [System.Serializable]
    private class FloorData
    {
        public List<Room> rooms;
        public List<Staircase> staircases;
        public List<ClusterData> clusters;
    }
    [System.Serializable]
    private class ClusterData { public List<Vector2Int> cells; }

    void Start()
    {
        player = GameObject.FindGameObjectWithTag("Player");
        InitializeGrid();
        GenerateDungeon();
        FindAllClusters();
        SerializeDungeonToJSON();
        LoadInitialClusters();
    }

    void Update()
    {
        if (player == null) return;
        Vector3 playerPos = player.transform.position;
        int playerFloor = Mathf.FloorToInt(playerPos.y / floorHeight);

        for (int f = 0; f < numberOfFloors; f++)
        {
            if (Mathf.Abs(f - playerFloor) > 1) continue;
            foreach (var cluster in floorClusters[f])
            {
                Vector2 clusterCenter = GetClusterCenter(cluster, f);
                float distance = Vector2.Distance(new Vector2(playerPos.x, playerPos.z), clusterCenter);
                if (distance <= clusterLoadRadius && !clusterObjects[f].ContainsKey(cluster))
                    StartCoroutine(LoadClusterAsync(cluster, f));
                else if (distance > clusterLoadRadius && clusterObjects[f].ContainsKey(cluster))
                    StartCoroutine(UnloadClusterAsync(cluster, f));
            }
        }
    }

    void InitializeGrid()
    {
        gridResolution = Mathf.CeilToInt(gridSize);
        grid = new GridCell[gridResolution, gridResolution, numberOfFloors];
        for (int f = 0; f < numberOfFloors; f++)
            for (int x = 0; x < gridResolution; x++)
                for (int y = 0; y < gridResolution; y++)
                    grid[x, y, f] = new GridCell();
    }

    void GenerateDungeon()
    {
        floors = new List<List<Room>>(numberOfFloors);
        triangulations = new List<List<Triangle>>(numberOfFloors);
        corridors = new List<List<(Vector2, Vector2)>>(numberOfFloors);
        corridorPaths = new List<List<List<Vector2Int>>>(numberOfFloors);
        staircases = new List<List<Staircase>>(numberOfFloors);

        for (int f = 0; f < numberOfFloors; f++)
        {
            floors.Add(new List<Room>());
            triangulations.Add(new List<Triangle>());
            corridors.Add(new List<(Vector2, Vector2)>());
            corridorPaths.Add(new List<List<Vector2Int>>());
            staircases.Add(new List<Staircase>());
            GenerateRooms(f);
            MarkRoomsOnGrid(f);
        }

        for (int f = 0; f < numberOfFloors - 1; f++)
            PlaceStaircases(f);

        for (int f = 0; f < numberOfFloors; f++)
        {
            DelaunayTriangulation(f);
            GenerateCorridors(f);
            PlaceDoorsBasedOnCorridors(f);
        }

        Visualize();
    }

    void GenerateRooms(int floor)
    {
        for (int i = 0; i < numberOfRoomsPerFloor; i++)
        {
            bool validPosition = false;
            int attempts = 0;
            Room newRoom = null;

            while (!validPosition && attempts < 100)
            {
                int width = Random.Range(Mathf.RoundToInt(roomSizeMin.x), Mathf.RoundToInt(roomSizeMax.x) + 1);
                int height = Random.Range(Mathf.RoundToInt(roomSizeMin.y), Mathf.RoundToInt(roomSizeMax.y) + 1);
                if (width % 2 != 0) width++;
                if (height % 2 != 0) height++;
                Vector2 size = new Vector2(width, height);
                int x = Random.Range(Mathf.RoundToInt(-gridSize / 2 + size.x / 2), Mathf.RoundToInt(gridSize / 2 - size.x / 2));
                int y = Random.Range(Mathf.RoundToInt(-gridSize / 2 + size.y / 2), Mathf.RoundToInt(gridSize / 2 - size.y / 2));
                Vector2 center = new Vector2(x, y);
                newRoom = new Room(center, size, bufferDistance, floor);
                validPosition = true;

                foreach (var room in floors[floor])
                    if (newRoom.bounds.Intersects(room.bounds))
                    {
                        validPosition = false;
                        break;
                    }

                attempts++;
            }

            if (validPosition && newRoom != null)
            {
                floors[floor].Add(newRoom);
                ApplyRoomCellularAutomaton(newRoom);
            }
        }
    }

    void ApplyRoomCellularAutomaton(Room room)
    {
        int width = Mathf.CeilToInt(room.size.x);
        int height = Mathf.CeilToInt(room.size.y);
        GridCell[,] roomGrid = room.roomGrid;

        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                roomGrid[x, y].roomPrefabType = RoomPrefabType.Floor;

        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                if (x == 0 || x == width - 1 || y == 0 || y == height - 1)
                    roomGrid[x, y].roomPrefabType = ((x == 0 && y == 0) || (x == 0 && y == height - 1) ||
                        (x == width - 1 && y == 0) || (x == width - 1 && y == height - 1)) ? RoomPrefabType.Corner : RoomPrefabType.WallFloor;
    }

    void PlaceStaircases(int floor)
    {
        int staircasesPlaced = 0;
        List<Room> availableRooms = floors[floor].OrderBy(x => Random.value).ToList();

        foreach (var room in availableRooms)
        {
            if (staircasesPlaced >= maxStaircases) break;
            Vector2Int tightMin = WorldToGrid(room.tightBounds.min);
            int width = Mathf.CeilToInt(room.size.x);
            int height = Mathf.CeilToInt(room.size.y);
            List<(Vector2Int, Vector2Int)> possibleStaircasePositions = new List<(Vector2Int, Vector2Int)>();

            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    if (!(x == 0 || x == width - 1 || y == 0 || y == height - 1)) continue;
                    if ((x == 0 && y == 0) || (x == 0 && y == height - 1) ||
                        (x == width - 1 && y == 0) || (x == width - 1 && y == height - 1)) continue;

                    Vector2Int worldGridPos = new Vector2Int(tightMin.x + x, tightMin.y + y);
                    if (!IsInGridBounds(worldGridPos, floor)) continue;

                    Vector2Int[] directions = { new Vector2Int(0, 1), new Vector2Int(0, -1), new Vector2Int(1, 0), new Vector2Int(-1, 0) };
                    foreach (var dir in directions)
                    {
                        Vector2Int adjacentPos = worldGridPos + dir;
                        if (!IsInGridBounds(adjacentPos, floor) || grid[adjacentPos.x, adjacentPos.y, floor].type != CellType.Empty) continue;
                        Vector2Int staircaseCell = worldGridPos + dir;
                        Vector2Int exitOffset = new Vector2Int(dir.y, -dir.x);
                        Vector2Int exitGridPos = staircaseCell + dir + exitOffset;
                        if (!IsInGridBounds(exitGridPos, floor + 1) || grid[exitGridPos.x, exitGridPos.y, floor + 1].type != CellType.Empty) continue;

                        Vector2 exitWorldPos = GridToWorld(exitGridPos);
                        bool isOutsideRoomBounds = true;
                        foreach (var upperRoom in floors[floor + 1])
                            if (upperRoom.bounds.Contains(exitWorldPos))
                            {
                                isOutsideRoomBounds = false;
                                break;
                            }

                        bool isFarEnough = true;
                        foreach (var upperRoom in floors[floor + 1])
                        {
                            Vector2Int tightMinUpper = WorldToGrid(upperRoom.tightBounds.min);
                            Vector2Int tightMaxUpper = WorldToGrid(upperRoom.tightBounds.max);
                            int minX = tightMinUpper.x, maxX = tightMaxUpper.x, minY = tightMinUpper.y, maxY = tightMaxUpper.y;
                            int dx = Mathf.Max(0, Mathf.Min(Mathf.Abs(exitGridPos.x - minX), Mathf.Abs(exitGridPos.x - maxX)));
                            int dy = Mathf.Max(0, Mathf.Min(Mathf.Abs(exitGridPos.y - minY), Mathf.Abs(exitGridPos.y - maxY)));
                            if (dx == 0) dx = Mathf.Abs(exitGridPos.x - (exitGridPos.x < minX ? minX : maxX));
                            if (dy == 0) dy = Mathf.Abs(exitGridPos.y - (exitGridPos.y < minY ? minY : maxY));
                            if (dx + dy < 2)
                            {
                                isFarEnough = false;
                                break;
                            }
                        }

                        if (isOutsideRoomBounds && isFarEnough)
                            possibleStaircasePositions.Add((worldGridPos, dir));
                    }
                }

            if (possibleStaircasePositions.Count > 0)
            {
                var (staircaseGridPos, direction) = possibleStaircasePositions[Random.Range(0, possibleStaircasePositions.Count)];
                Vector2 staircaseWorldPos = GridToWorld(staircaseGridPos);
                Vector2Int staircaseCell = staircaseGridPos + direction;
                Vector2Int exitOffset = new Vector2Int(direction.y, -direction.x);
                Vector2Int exitGridPos = staircaseCell + direction + exitOffset;

                Staircase staircase = new Staircase
                {
                    position = staircaseWorldPos,
                    gridPosition = staircaseGridPos,
                    fromFloor = floor,
                    toFloor = floor + 1,
                    direction = direction,
                    attachedRoom = room,
                    exitGridPosition = exitGridPos
                };

                Vector2Int roomGridPos = staircaseGridPos - tightMin;
                room.roomGrid[roomGridPos.x, roomGridPos.y].roomPrefabType = RoomPrefabType.StaircaseFloor;
                grid[staircaseGridPos.x, staircaseGridPos.y, floor].type = CellType.Room;
                grid[staircaseGridPos.x, staircaseGridPos.y, floor].roomPrefabType = RoomPrefabType.StaircaseFloor;
                grid[staircaseCell.x, staircaseCell.y, floor].type = CellType.Staircase;
                grid[exitGridPos.x, exitGridPos.y, floor + 1].type = CellType.Corridor;
                staircases[floor].Add(staircase);
                staircasesPlaced++;
            }
        }
    }

    void PlaceDoorsBasedOnCorridors(int floor)
    {
        foreach (var room in floors[floor])
        {
            Vector2Int tightMin = WorldToGrid(room.tightBounds.min);
            int width = Mathf.CeilToInt(room.size.x);
            int height = Mathf.CeilToInt(room.size.y);

            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    if (!(x == 0 || x == width - 1 || y == 0 || y == height - 1)) continue;
                    if ((x == 0 && y == 0) || (x == 0 && y == height - 1) ||
                        (x == width - 1 && y == 0) || (x == width - 1 && y == height - 1)) continue;

                    Vector2Int worldGridPos = new Vector2Int(tightMin.x + x, tightMin.y + y);
                    if (!IsInGridBounds(worldGridPos, floor)) continue;

                    Vector2Int[] neighbors = {
                        new Vector2Int(worldGridPos.x, worldGridPos.y + 1),
                        new Vector2Int(worldGridPos.x, worldGridPos.y - 1),
                        new Vector2Int(worldGridPos.x + 1, worldGridPos.y),
                        new Vector2Int(worldGridPos.x - 1, worldGridPos.y)
                    };

                    bool placeDoor = false;
                    foreach (var neighbor in neighbors)
                        if (IsInGridBounds(neighbor, floor) && grid[neighbor.x, neighbor.y, floor].type == CellType.CorridorNearRoom)
                        {
                            placeDoor = true;
                            break;
                        }

                    if (placeDoor)
                        room.roomGrid[x, y].roomPrefabType = RoomPrefabType.DoorWall;
                }

            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    Vector2Int worldGridPos = new Vector2Int(tightMin.x + x, tightMin.y + y);
                    if (IsInGridBounds(worldGridPos, floor))
                        grid[worldGridPos.x, worldGridPos.y, floor].roomPrefabType = room.roomGrid[x, y].roomPrefabType;
                }
        }
    }

    void MarkRoomsOnGrid(int floor)
    {
        foreach (var room in floors[floor])
        {
            Vector2Int tightMin = WorldToGrid(room.tightBounds.min);
            Vector2Int tightMax = WorldToGrid(room.tightBounds.max);
            int width = Mathf.CeilToInt(room.size.x);
            int height = Mathf.CeilToInt(room.size.y);

            for (int x = Mathf.Max(0, tightMin.x); x <= Mathf.Min(gridResolution - 1, tightMax.x); x++)
                for (int y = Mathf.Max(0, tightMin.y); y <= Mathf.Min(gridResolution - 1, tightMax.y); y++)
                {
                    int roomX = x - tightMin.x;
                    int roomY = y - tightMin.y;
                    if (roomX >= 0 && roomX < width && roomY >= 0 && roomY < height)
                    {
                        grid[x, y, floor].type = CellType.Room;
                        grid[x, y, floor].roomPrefabType = room.roomGrid[roomX, roomY].roomPrefabType;
                    }
                }
        }
    }

    void DelaunayTriangulation(int floor)
    {
        Vector2 min = new Vector2(-gridSize, -gridSize);
        Vector2 max = new Vector2(gridSize, gridSize);
        Vector2 super1 = new Vector2(min.x - 100, min.y - 100);
        Vector2 super2 = new Vector2(max.x + 100, min.y - 100);
        Vector2 super3 = new Vector2(0, max.y + 100);
        triangulations[floor].Add(new Triangle(super1, super2, super3));

        List<Vector2> points = floors[floor].Select(r => r.center).ToList();
        if (floor > 0)
            foreach (var staircase in staircases[floor - 1])
                points.Add(GridToWorld(staircase.exitGridPosition));

        foreach (var point in points)
        {
            List<Triangle> badTriangles = triangulations[floor].Where(t => t.IsPointInCircumcircle(point)).ToList();
            List<(Vector2, Vector2)> polygon = new List<(Vector2, Vector2)>();

            foreach (var triangle in badTriangles)
            {
                var edges = new[] { (triangle.p1, triangle.p2), (triangle.p2, triangle.p3), (triangle.p3, triangle.p1) };
                foreach (var edge in edges)
                {
                    bool isShared = badTriangles.Any(other => other != triangle &&
                        ((other.p1 == edge.Item1 && other.p2 == edge.Item2) ||
                         (other.p1 == edge.Item2 && other.p2 == edge.Item1) ||
                         (other.p2 == edge.Item1 && other.p3 == edge.Item2) ||
                         (other.p2 == edge.Item2 && other.p3 == edge.Item1) ||
                         (other.p3 == edge.Item1 && other.p1 == edge.Item2) ||
                         (other.p3 == edge.Item2 && other.p1 == edge.Item1)));
                    if (!isShared) polygon.Add(edge);
                }
            }

            triangulations[floor].RemoveAll(t => badTriangles.Contains(t));
            foreach (var edge in polygon)
                triangulations[floor].Add(new Triangle(edge.Item1, edge.Item2, point));
        }

        triangulations[floor].RemoveAll(t => t.p1 == super1 || t.p1 == super2 || t.p1 == super3 ||
                                            t.p2 == super1 || t.p2 == super2 || t.p2 == super3 ||
                                            t.p3 == super1 || t.p3 == super2 || t.p3 == super3);
    }

    void GenerateCorridors(int floor)
    {
        HashSet<(Vector2, Vector2)> edges = new HashSet<(Vector2, Vector2)>();
        foreach (var triangle in triangulations[floor])
        {
            edges.Add((triangle.p1, triangle.p2));
            edges.Add((triangle.p2, triangle.p3));
            edges.Add((triangle.p3, triangle.p1));
        }

        List<(Vector2, Vector2)> mst = new List<(Vector2, Vector2)>();
        HashSet<Vector2> visited = new HashSet<Vector2>();
        List<(Vector2, Vector2, float)> candidates = new List<(Vector2, Vector2, float)>();
        List<Vector2> connectionPoints = floors[floor].Select(r => r.center).ToList();
        List<Vector2> staircaseEntryPoints = new List<Vector2>();

        if (floor > 0)
            foreach (var staircase in staircases[floor - 1])
            {
                Vector2 exitPoint = GridToWorld(staircase.exitGridPosition);
                staircaseEntryPoints.Add(exitPoint);
                connectionPoints.Add(exitPoint);
            }

        if (connectionPoints.Count == 0) return;

        visited.Add(connectionPoints[0]);
        foreach (var edge in edges)
            if (edge.Item1 == connectionPoints[0] || edge.Item2 == connectionPoints[0])
                candidates.Add((edge.Item1, edge.Item2, Vector2.Distance(edge.Item1, edge.Item2)));

        while (visited.Count < connectionPoints.Count && candidates.Count > 0)
        {
            candidates.Sort((a, b) => a.Item3.CompareTo(b.Item3));
            var (u, v, _) = candidates[0];
            candidates.RemoveAt(0);
            Vector2 newVertex = visited.Contains(u) ? v : u;

            if (!visited.Contains(newVertex))
            {
                mst.Add((u, v));
                visited.Add(newVertex);
                foreach (var edge in edges)
                    if ((edge.Item1 == newVertex && !visited.Contains(edge.Item2)) || (edge.Item2 == newVertex && !visited.Contains(edge.Item1)))
                        candidates.Add((edge.Item1, edge.Item2, Vector2.Distance(edge.Item1, edge.Item2)));
            }
        }

        corridors[floor].AddRange(mst);

        if (floor > 0)
            foreach (var staircaseEntry in staircaseEntryPoints)
                if (!mst.Any(edge => edge.Item1 == staircaseEntry || edge.Item2 == staircaseEntry))
                {
                    Vector2 nearestRoom = floors[floor].Select(r => r.center).OrderBy(p => Vector2.Distance(p, staircaseEntry)).FirstOrDefault();
                    if (nearestRoom != default)
                        corridors[floor].Add((staircaseEntry, nearestRoom));
                }

        foreach (var edge in edges)
            if (!mst.Contains(edge) && !mst.Contains((edge.Item2, edge.Item1)) && Random.value < extraEdgeProbability)
                corridors[floor].Add(edge);

        corridorPaths[floor].Clear();
        foreach (var corridor in corridors[floor])
        {
            Vector2Int start = WorldToGrid(corridor.Item1);
            Vector2Int end = WorldToGrid(corridor.Item2);
            List<Vector2Int> path = FindPathAStar(start, end, floor);
            if (path != null)
                corridorPaths[floor].Add(path);
        }

        ApplyCellularAutomaton(floor);

        foreach (var path in corridorPaths[floor])
            for (int i = 0; i < path.Count; i++)
            {
                var cell = path[i];
                if (!IsInGridBounds(cell, floor)) continue;
                Vector2 worldPos = GridToWorld(cell);
                bool isInRoomTightBounds = false;
                bool isInRoomBuffer = false;
                Room currentRoom = null;

                foreach (var room in floors[floor])
                {
                    if (room.tightBounds.Contains(worldPos))
                    {
                        isInRoomTightBounds = true;
                        break;
                    }
                    if (room.bounds.Contains(worldPos) && !room.tightBounds.Contains(worldPos))
                    {
                        isInRoomBuffer = true;
                        currentRoom = room;
                    }
                }

                if (isInRoomBuffer)
                {
                    bool isCorridorNearRoom = false;
                    if (i > 0 && !currentRoom.bounds.Contains(GridToWorld(path[i - 1])))
                        isCorridorNearRoom = true;
                    if (i < path.Count - 1 && !isCorridorNearRoom && !currentRoom.bounds.Contains(GridToWorld(path[i + 1])))
                        isCorridorNearRoom = true;

                    if (isCorridorNearRoom)
                    {
                        grid[cell.x, cell.y, floor].type = CellType.CorridorNearRoom;
                        continue;
                    }
                }

                if (!isInRoomTightBounds)
                    grid[cell.x, cell.y, floor].type = CellType.Corridor;
            }
    }

    List<Vector2Int> FindPathAStar(Vector2Int start, Vector2Int goal, int floor)
    {
        List<Node> openList = new List<Node>();
        HashSet<Vector2Int> closedList = new HashSet<Vector2Int>();
        openList.Add(new Node(start, floor, 0, ManhattanDistance(start, goal), null));

        while (openList.Count > 0)
        {
            Node current = openList.OrderBy(n => n.FCost).ThenBy(n => n.hCost).First();
            openList.Remove(current);

            if (current.position == goal)
            {
                List<Vector2Int> path = new List<Vector2Int>();
                Node node = current;
                while (node != null)
                {
                    path.Add(node.position);
                    node = node.parent;
                }
                path.Reverse();
                return path;
            }

            closedList.Add(current.position);
            Vector2Int[] neighbors = { new Vector2Int(0, 1), new Vector2Int(0, -1), new Vector2Int(1, 0), new Vector2Int(-1, 0) };

            foreach (var offset in neighbors)
            {
                Vector2Int neighborPos = current.position + offset;
                if (!IsInGridBounds(neighborPos, floor) || closedList.Contains(neighborPos)) continue;
                float moveCost = GetCellCost(neighborPos, floor);
                if (moveCost >= 1000) continue;
                float tentativeGCost = current.gCost + moveCost;
                Node neighborNode = openList.Find(n => n.position == neighborPos && n.floor == floor);

                if (neighborNode == null)
                {
                    neighborNode = new Node(neighborPos, floor, tentativeGCost, ManhattanDistance(neighborPos, goal), current);
                    openList.Add(neighborNode);
                }
                else if (tentativeGCost < neighborNode.gCost)
                {
                    neighborNode.gCost = tentativeGCost;
                    neighborNode.parent = current;
                }
            }
        }

        return null;
    }

    float GetCellCost(Vector2Int pos, int floor)
    {
        if (!IsInGridBounds(pos, floor)) return 1000f;
        if (grid[pos.x, pos.y, floor].type == CellType.Room) return 100f;

        foreach (var room in floors[floor])
            if (room.bounds.Contains(GridToWorld(pos)) && !room.tightBounds.Contains(GridToWorld(pos)))
                return 50f;

        if (grid[pos.x, pos.y, floor].type == CellType.Corridor || grid[pos.x, pos.y, floor].type == CellType.Staircase)
            return 0.1f;

        if (floor > 0)
            foreach (var staircase in staircases[floor - 1])
            {
                Vector2Int exitPos = staircase.exitGridPosition;
                Vector2Int exitDirection = new Vector2Int(staircase.direction.y, -staircase.direction.x);
                if (pos == exitPos + exitDirection) return 0.01f;
                if (pos == exitPos + new Vector2Int(-exitDirection.x, -exitDirection.y) ||
                    pos == exitPos + new Vector2Int(staircase.direction.x, staircase.direction.y) ||
                    pos == exitPos + new Vector2Int(-staircase.direction.x, -staircase.direction.y))
                    return 10f;
                if (staircase.exitGridPosition == pos) return 0.05f;
            }

        return 1f;
    }

    float ManhattanDistance(Vector2Int a, Vector2Int b) => Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);

    Vector2Int WorldToGrid(Vector2 worldPos) => new Vector2Int(Mathf.FloorToInt(worldPos.x + gridSize / 2), Mathf.FloorToInt(worldPos.y + gridSize / 2));

    Vector2 GridToWorld(Vector2Int gridPos) => new Vector2(gridPos.x - gridSize / 2 + 0.5f, gridPos.y - gridSize / 2 + 0.5f);

    bool IsInGridBounds(Vector2Int pos, int floor) => pos.x >= 0 && pos.x < gridResolution && pos.y >= 0 && pos.y < gridResolution && floor >= 0 && floor < numberOfFloors;

    void Visualize() { }

    bool IsDoubleCorridor(int x, int z, int floor)
    {
        bool left = IsCorridorOrCorridorNearRoom(x - 1, z, floor);
        bool right = IsCorridorOrCorridorNearRoom(x + 1, z, floor);
        bool up = IsCorridorOrCorridorNearRoom(x, z + 1, floor);
        bool down = IsCorridorOrCorridorNearRoom(x, z - 1, floor);

        return (left && right) || (left && IsCorridorOrCorridorNearRoom(x - 2, z, floor)) || (right && IsCorridorOrCorridorNearRoom(x + 2, z, floor)) ||
               (up && down) || (up && IsCorridorOrCorridorNearRoom(x, z + 2, floor)) || (down && IsCorridorOrCorridorNearRoom(x, z - 2, floor));
    }

    bool IsCorridorOrCorridorNearRoom(int x, int z, int floor) => IsInGridBounds(new Vector2Int(x, z), floor) &&
        (grid[x, z, floor].type == CellType.Corridor || grid[x, z, floor].type == CellType.CorridorNearRoom);

    void ApplyCellularAutomaton(int floor)
    {
        int iterations = 3, minNeighbors = 3, corridorWidth = 2;
        GridCell[,,] newGrid = new GridCell[gridResolution, gridResolution, numberOfFloors];

        for (int x = 0; x < gridResolution; x++)
            for (int y = 0; y < gridResolution; y++)
                newGrid[x, y, floor] = new GridCell { type = grid[x, y, floor].type, roomPrefabType = grid[x, y, floor].roomPrefabType };

        for (int iter = 0; iter < iterations; iter++)
        {
            for (int x = 0; x < gridResolution; x++)
                for (int y = 0; y < gridResolution; y++)
                {
                    if (grid[x, y, floor].type == CellType.Room || grid[x, y, floor].type == CellType.Staircase) continue;
                    int neighborCount = CountCorridorNeighbors(x, y, floor);
                    newGrid[x, y, floor].type = (grid[x, y, floor].type == CellType.Corridor && neighborCount >= minNeighbors) ? CellType.Corridor :
                        (grid[x, y, floor].type == CellType.Empty && neighborCount >= minNeighbors + 1) ? CellType.Corridor :
                        (grid[x, y, floor].type != CellType.Room && grid[x, y, floor].type != CellType.Staircase) ? CellType.Empty : newGrid[x, y, floor].type;
                }

            for (int x = 0; x < gridResolution; x++)
                for (int y = 0; y < gridResolution; y++)
                    grid[x, y, floor].type = newGrid[x, y, floor].type;
        }

        for (int x = 0; x < gridResolution; x++)
            for (int y = 0; y < gridResolution; y++)
                newGrid[x, y, floor] = new GridCell { type = grid[x, y, floor].type, roomPrefabType = grid[x, y, floor].roomPrefabType };

        for (int x = 0; x < gridResolution; x++)
            for (int y = 0; y < gridResolution; y++)
                if (grid[x, y, floor].type == CellType.Corridor)
                    for (int dx = -corridorWidth / 2; dx <= corridorWidth / 2; dx++)
                        for (int dy = -corridorWidth / 2; dy <= corridorWidth / 2; dy++)
                        {
                            int nx = x + dx, ny = y + dy;
                            if (IsInGridBounds(new Vector2Int(nx, ny), floor) && grid[nx, ny, floor].type != CellType.Room &&
                                grid[nx, ny, floor].type != CellType.CorridorNearRoom && grid[nx, ny, floor].type != CellType.Staircase)
                                newGrid[nx, ny, floor].type = CellType.Corridor;
                        }

        for (int x = 0; x < gridResolution; x++)
            for (int y = 0; y < gridResolution; y++)
                grid[x, y, floor].type = newGrid[x, y, floor].type;
    }

    int CountCorridorNeighbors(int x, int y, int floor)
    {
        int count = 0;
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                if (!(dx == 0 && dy == 0) && IsInGridBounds(new Vector2Int(x + dx, y + dy), floor) &&
                    grid[x + dx, y + dy, floor].type == CellType.Corridor)
                    count++;
        return count;
    }

    void FindAllClusters()
    {
        for (int f = 0; f < numberOfFloors; f++)
        {
            floorClusters[f] = FindClusters(f);
            clusterObjects[f] = new Dictionary<List<Vector2Int>, GameObject>();
        }
    }

    List<List<Vector2Int>> FindClusters(int floor)
    {
        List<List<Vector2Int>> clusters = new List<List<Vector2Int>>();
        bool[,] visited = new bool[gridResolution, gridResolution];

        for (int x = 0; x < gridResolution; x++)
            for (int y = 0; y < gridResolution; y++)
                if (!visited[x, y] && (grid[x, y, floor].type == CellType.Room || grid[x, y, floor].type == CellType.Corridor ||
                    grid[x, y, floor].type == CellType.CorridorNearRoom || grid[x, y, floor].type == CellType.Staircase))
                {
                    List<Vector2Int> cluster = new List<Vector2Int>();
                    BFSClusterSearch(new Vector2Int(x, y), floor, visited, cluster);
                    clusters.Add(cluster);
                }

        return clusters;
    }

    void BFSClusterSearch(Vector2Int start, int floor, bool[,] visited, List<Vector2Int> cluster)
    {
        Queue<Vector2Int> queue = new Queue<Vector2Int>();
        queue.Enqueue(start);
        visited[start.x, start.y] = true;
        Vector2Int[] directions = { new Vector2Int(0, 1), new Vector2Int(0, -1), new Vector2Int(1, 0), new Vector2Int(-1, 0) };

        while (queue.Count > 0)
        {
            Vector2Int current = queue.Dequeue();
            cluster.Add(current);
            foreach (var dir in directions)
            {
                Vector2Int neighbor = current + dir;
                if (IsInGridBounds(neighbor, floor) && !visited[neighbor.x, neighbor.y] &&
                    (grid[neighbor.x, neighbor.y, floor].type == CellType.Room || grid[neighbor.x, neighbor.y, floor].type == CellType.Corridor ||
                     grid[neighbor.x, neighbor.y, floor].type == CellType.CorridorNearRoom || grid[neighbor.x, neighbor.y, floor].type == CellType.Staircase))
                {
                    queue.Enqueue(neighbor);
                    visited[neighbor.x, neighbor.y] = true;
                }
            }
        }
    }

    void SerializeDungeonToJSON()
    {
        DungeonData dungeonData = new DungeonData { floors = new List<FloorData>() };
        for (int f = 0; f < numberOfFloors; f++)
            dungeonData.floors.Add(new FloorData
            {
                rooms = floors[f],
                staircases = staircases[f],
                clusters = floorClusters[f].Select(c => new ClusterData { cells = c }).ToList()
            });

        File.WriteAllText(Application.persistentDataPath + "/dungeon.json", JsonUtility.ToJson(dungeonData, true));
    }

    void LoadInitialClusters()
    {
        if (player == null) return;
        Vector3 playerPos = player.transform.position;
        int playerFloor = Mathf.FloorToInt(playerPos.y / floorHeight);

        for (int f = 0; f < numberOfFloors; f++)
            if (Mathf.Abs(f - playerFloor) <= 1)
                foreach (var cluster in floorClusters[f])
                {
                    Vector2 clusterCenter = GetClusterCenter(cluster, f);
                    if (Vector2.Distance(new Vector2(playerPos.x, playerPos.z), clusterCenter) <= clusterLoadRadius)
                    {
                        GameObject clusterObj = VisualizeCluster(cluster, f);
                        clusterObjects[f][cluster] = clusterObj;
                    }
                }
    }

    System.Collections.IEnumerator LoadClusterAsync(List<Vector2Int> cluster, int floor)
    {
        clusterObjects[floor][cluster] = VisualizeCluster(cluster, floor);
        yield return null;
    }

    System.Collections.IEnumerator UnloadClusterAsync(List<Vector2Int> cluster, int floor)
    {
        if (clusterObjects[floor].ContainsKey(cluster))
        {
            Destroy(clusterObjects[floor][cluster]);
            clusterObjects[floor].Remove(cluster);
        }
        yield return null;
    }

    Vector2 GetClusterCenter(List<Vector2Int> cluster, int floor) => cluster.Aggregate(Vector2.zero, (sum, cell) => sum + GridToWorld(cell)) / cluster.Count;

    GameObject VisualizeCluster(List<Vector2Int> cluster, int floor)
    {
        GameObject clusterObj = new GameObject("Cluster_F" + floor);
        HashSet<(int, int)> visualizedCells = new HashSet<(int, int)>();
        float yOffset = floor * floorHeight;

        foreach (var cell in cluster)
        {
            int x = cell.x, z = cell.y;
            if (!IsInGridBounds(cell, floor) || visualizedCells.Contains((x, z))) continue;
            visualizedCells.Add((x, z));
            Vector2 worldPos = GridToWorld(cell);

            if (grid[x, z, floor].type == CellType.CorridorNearRoom && corridorNearRoomPrefab != null)
            {
                bool corridorUp = IsInGridBounds(new Vector2Int(x, z + 1), floor) && grid[x, z + 1, floor].type == CellType.Corridor;
                bool corridorDown = IsInGridBounds(new Vector2Int(x, z - 1), floor) && grid[x, z - 1, floor].type == CellType.Corridor;
                bool corridorLeft = IsInGridBounds(new Vector2Int(x - 1, z), floor) && grid[x - 1, z, floor].type == CellType.Corridor;
                bool corridorRight = IsInGridBounds(new Vector2Int(x + 1, z), floor) && grid[x + 1, z, floor].type == CellType.Corridor;
                Quaternion rotation = (corridorUp || corridorDown) ? Quaternion.Euler(0, 0, 0) : Quaternion.Euler(0, 90, 0);
                GameObject instance = Instantiate(corridorNearRoomPrefab, new Vector3(worldPos.x, yOffset, worldPos.y), rotation);
                instance.transform.parent = clusterObj.transform;
            }
            else if (grid[x, z, floor].type == CellType.Room)
            {
                GameObject prefabToUse = null;
                Quaternion rotation = Quaternion.identity;
                bool isNorthWall = z == gridResolution - 1 || (IsInGridBounds(new Vector2Int(x, z + 1), floor) && grid[x, z + 1, floor].type != CellType.Room);
                bool isSouthWall = z == 0 || (IsInGridBounds(new Vector2Int(x, z - 1), floor) && grid[x, z - 1, floor].type != CellType.Room);
                bool isEastWall = x == gridResolution - 1 || (IsInGridBounds(new Vector2Int(x + 1, z), floor) && grid[x + 1, z, floor].type != CellType.Room);
                bool isWestWall = x == 0 || (IsInGridBounds(new Vector2Int(x - 1, z), floor) && grid[x - 1, z, floor].type != CellType.Room);

                switch (grid[x, z, floor].roomPrefabType)
                {
                    case RoomPrefabType.WallFloor:
                        prefabToUse = wallFloorPrefab;
                        rotation = isNorthWall ? Quaternion.Euler(0, 0, 0) : isEastWall ? Quaternion.Euler(0, 90, 0) :
                            isSouthWall ? Quaternion.Euler(0, 180, 0) : Quaternion.Euler(0, 270, 0);
                        break;
                    case RoomPrefabType.Floor:
                    case RoomPrefabType.StaircaseFloor:
                        prefabToUse = floorPrefab;
                        break;
                    case RoomPrefabType.Corner:
                        prefabToUse = cornerPrefab;
                        rotation = (isNorthWall && isWestWall) ? Quaternion.Euler(0, 270, 0) : (isNorthWall && isEastWall) ? Quaternion.Euler(0, 0, 0) :
                            (isSouthWall && isWestWall) ? Quaternion.Euler(0, 180, 0) : Quaternion.Euler(0, 90, 0);
                        break;
                    case RoomPrefabType.DoorWall:
                        prefabToUse = doorWallPrefab;
                        rotation = isNorthWall ? Quaternion.Euler(0, 0, 0) : isEastWall ? Quaternion.Euler(0, 90, 0) :
                            isSouthWall ? Quaternion.Euler(0, 180, 0) : Quaternion.Euler(0, 270, 0);
                        break;
                }

                if (prefabToUse != null)
                {
                    GameObject instance = Instantiate(prefabToUse, new Vector3(worldPos.x, yOffset, worldPos.y), rotation);
                    instance.transform.parent = clusterObj.transform;
                }
            }
            else if (grid[x, z, floor].type == CellType.Corridor)
            {
                if (floor > 0 && staircases[floor - 1].Any(s => s.exitGridPosition == cell)) continue;
                bool isDoubleCorridor = IsDoubleCorridor(x, z, floor);
                bool up = IsCorridorOrCorridorNearRoom(x, z + 1, floor);
                bool down = IsCorridorOrCorridorNearRoom(x, z - 1, floor);
                bool left = IsCorridorOrCorridorNearRoom(x - 1, z, floor);
                bool right = IsCorridorOrCorridorNearRoom(x + 1, z, floor);
                int neighborCount = (up ? 1 : 0) + (down ? 1 : 0) + (left ? 1 : 0) + (right ? 1 : 0);
                Quaternion rotation = Quaternion.identity;
                GameObject prefabToUse = isDoubleCorridor ? corridorPrefab : corridorPrefab;

                if (neighborCount == 2)
                {
                    if (up && down)
                        rotation = Quaternion.Euler(0, 0, 0);
                    else if (left && right)
                        rotation = Quaternion.Euler(0, 90, 0);
                    else
                    {
                        prefabToUse = isDoubleCorridor ? multiRoadCorridorJointPrefab : corridorJointPrefab;
                        rotation = (up && right) ? Quaternion.Euler(0, 0, 0) : (up && left) ? Quaternion.Euler(0, 270, 0) :
                            (down && right) ? Quaternion.Euler(0, 90, 0) : Quaternion.Euler(0, 180, 0);
                    }
                }
                else if (neighborCount == 3)
                {
                    prefabToUse = isDoubleCorridor ? multiRoadTCorridorPrefab : tCorridorPrefab;
                    rotation = (up && left && right) ? Quaternion.Euler(0, 270, 0) : (down && left && right) ? Quaternion.Euler(0, 90, 0) :
                        (up && down && right) ? Quaternion.Euler(0, 0, 0) : Quaternion.Euler(0, 180, 0);
                }
                else if (neighborCount == 4)
                {
                    prefabToUse = isDoubleCorridor ? crossCorridorPrefab : crossCorridorPrefab;
                    rotation = Quaternion.Euler(0, 0, 0);
                }

                if (prefabToUse != null)
                {
                    GameObject instance = Instantiate(prefabToUse, new Vector3(worldPos.x, yOffset, worldPos.y), rotation);
                    instance.transform.parent = clusterObj.transform;
                }
            }
            else if (grid[x, z, floor].type == CellType.Staircase)
            {
                Staircase staircase = staircases[floor].Find(s => s.gridPosition + s.direction == cell);
                if (staircase != null)
                {
                    Quaternion rotation = staircase.direction == new Vector2Int(1, 0) ? Quaternion.Euler(0, 90, 0) :
                        staircase.direction == new Vector2Int(-1, 0) ? Quaternion.Euler(0, 270, 0) :
                        staircase.direction == new Vector2Int(0, 1) ? Quaternion.Euler(0, 0, 0) : Quaternion.Euler(0, 180, 0);
                    GameObject instance = Instantiate(staircasePrefab, new Vector3(worldPos.x, yOffset, worldPos.y), rotation);
                    instance.transform.parent = clusterObj.transform;
                }
            }
        }

        return clusterObj;
    }

    GameObject CreateCluster(Vector3 position)
    {
        GameObject cluster = new GameObject("DungeonCluster");
        cluster.transform.position = position;
        GameObject room = GameObject.CreatePrimitive(PrimitiveType.Cube);
        room.transform.parent = cluster.transform;
        room.transform.localPosition = Vector3.zero;
        return cluster;
    }
}