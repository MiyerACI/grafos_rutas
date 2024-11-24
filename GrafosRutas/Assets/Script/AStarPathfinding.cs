using UnityEngine;
using System.Collections.Generic;

public class AStarPathfinding : MonoBehaviour
{
    // Configuración pública
    public Transform targetPoint;              // Punto de destino
    public LayerMask walkableLayer;            // Capa de objetos caminables
    public float nodeRadius = 5f;              // Radio de cada nodo en la cuadrícula
    public float maxSlope = 45f;               // Pendiente máxima que se considera caminable
    public float characterHeight = 2f;         // Altura del personaje
    public Color pathColor = Color.green;      // Color para visualizar el camino
    public float pathDuration = 5f;            // Duración de la visualización del camino
    public float gridHeightOffset = 0f;        // Desplazamiento de altura para el grid

    // Variables privadas
    private Node[,] grid;                      // Cuadrícula de nodos
    private int gridSizeX, gridSizeZ;          // Tamaño de la cuadrícula
    private Terrain terrain;                   // Referencia al terreno
    private Vector3 terrainSize;               // Tamaño del terreno
    private float executionTime;
    private float pathLength;

    // Inicialización
    private void Awake()
    {
        InicializarTerreno();
        CrearCuadricula();
    }

    // Inicializa la referencia al terreno y calcula su tamaño
    private void InicializarTerreno()
    {
        terrain = Terrain.activeTerrain;
        if (terrain == null)
        {
            Debug.LogError("No se encontró un terreno activo en la escena.");
            return;
        }
        terrainSize = terrain.terrainData.size;
        gridSizeX = Mathf.RoundToInt(terrainSize.x / nodeRadius);
        gridSizeZ = Mathf.RoundToInt(terrainSize.z / nodeRadius);
    }

    // Clase interna para representar un nodo en la cuadrícula
    private class Node
    {
        public bool walkable;
        public Vector3 worldPosition;
        public int gridX;
        public int gridZ;
        public float gCost;
        public float hCost;
        public Node parent;

        public float fCost => gCost + hCost;

        public Node(bool _walkable, Vector3 _worldPosition, int _gridX, int _gridZ)
        {
            walkable = _walkable;
            worldPosition = _worldPosition;
            gridX = _gridX;
            gridZ = _gridZ;
        }
    }

    // Crea la cuadrícula de nodos
    private void CrearCuadricula()
    {
        grid = new Node[gridSizeX, gridSizeZ];

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int z = 0; z < gridSizeZ; z++)
            {
                Vector3 worldPoint = new Vector3(x * nodeRadius, 0, z * nodeRadius);
                float terrainHeight = terrain.SampleHeight(worldPoint);
                worldPoint.y = terrainHeight + gridHeightOffset;
                bool walkable = EsCaminable(worldPoint);
                grid[x, z] = new Node(walkable, worldPoint, x, z);
            }
        }
    }

    // Determina si un punto es caminable
    private bool EsCaminable(Vector3 worldPoint)
    {
        // Verifica obstáculos
        if (Physics.CheckSphere(worldPoint + Vector3.up * characterHeight * 0.5f, nodeRadius * 0.5f, ~walkableLayer))
        {
            return false;
        }

        // Verifica pendiente
        Vector3 normal = terrain.terrainData.GetInterpolatedNormal(worldPoint.x / terrainSize.x, worldPoint.z / terrainSize.z);
        float slope = Vector3.Angle(normal, Vector3.up);
        return slope <= maxSlope;
    }

    // Encuentra un camino desde la posición inicial hasta el punto objetivo
    public List<Vector3> EncontrarCamino(Vector3 startPosition)
    {
        if (terrain == null || grid == null)
        {
            Debug.LogError("El terreno o la cuadrícula no están inicializados.");
            return null;
        }

        System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
        stopwatch.Start();

        Vector3 clampedStart = LimitarPosicionAlTerreno(startPosition);
        Vector3 clampedTarget = LimitarPosicionAlTerreno(targetPoint.position);

        Node startNode = EncontrarNodoCaminableCercano(clampedStart);
        Node targetNode = EncontrarNodoCaminableCercano(clampedTarget);

        if (startNode == null || targetNode == null)
        {
            Debug.LogWarning("No se pudo encontrar un nodo inicial o final caminable.");
            return null;
        }

        // Implementación del algoritmo A*
        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost || openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                List<Vector3> path = ReconstruirCamino(startNode, targetNode);
                DibujarCamino(path);

                stopwatch.Stop();
                executionTime = stopwatch.ElapsedMilliseconds / 1000f;
                pathLength = CalcularLongitudRuta(path);

                Debug.Log($"Tiempo de ejecución: {executionTime} segundos");
                Debug.Log($"Longitud de la ruta: {pathLength} unidades (metros)");

                return path;
            }

            foreach (Node neighbor in ObtenerVecinos(currentNode))
            {
                if (!neighbor.walkable || closedSet.Contains(neighbor))
                {
                    continue;
                }

                float newCostToNeighbor = currentNode.gCost + ObtenerDistancia(currentNode, neighbor);
                if (newCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newCostToNeighbor;
                    neighbor.hCost = ObtenerDistancia(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }

        Debug.LogWarning("No se pudo encontrar un camino.");
        return null;
    }

    // Encuentra el nodo caminable más cercano a una posición dada
    private Node EncontrarNodoCaminableCercano(Vector3 position)
    {
        int centerX = Mathf.RoundToInt(position.x / nodeRadius);
        int centerZ = Mathf.RoundToInt(position.z / nodeRadius);

        Node closestNode = null;
        float closestDistance = float.MaxValue;

        for (int x = -2; x <= 2; x++)
        {
            for (int z = -2; z <= 2; z++)
            {
                int checkX = centerX + x;
                int checkZ = centerZ + z;

                if (checkX >= 0 && checkX < gridSizeX && checkZ >= 0 && checkZ < gridSizeZ)
                {
                    Node node = grid[checkX, checkZ];
                    if (node.walkable)
                    {
                        float distance = Vector3.Distance(position, node.worldPosition);
                        if (distance < closestDistance)
                        {
                            closestNode = node;
                            closestDistance = distance;
                        }
                    }
                }
            }
        }

        return closestNode;
    }

    // Reconstruye el camino desde el nodo inicial hasta el final
    private List<Vector3> ReconstruirCamino(Node startNode, Node endNode)
    {
        List<Vector3> path = new List<Vector3>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode.worldPosition);
            currentNode = currentNode.parent;
        }

        path.Reverse();
        return path;
    }

    // Obtiene los nodos vecinos de un nodo dado
    private List<Node> ObtenerVecinos(Node node)
    {
        List<Node> neighbors = new List<Node>();

        for (int x = -1; x <= 1; x++)
        {
            for (int z = -1; z <= 1; z++)
            {
                if (x == 0 && z == 0) continue;

                int checkX = node.gridX + x;
                int checkZ = node.gridZ + z;

                if (checkX >= 0 && checkX < gridSizeX && checkZ >= 0 && checkZ < gridSizeZ)
                {
                    neighbors.Add(grid[checkX, checkZ]);
                }
            }
        }

        return neighbors;
    }

    // Calcula la distancia entre dos nodos
    private float ObtenerDistancia(Node a, Node b)
    {
        float dstX = Mathf.Abs(a.gridX - b.gridX);
        float dstZ = Mathf.Abs(a.gridZ - b.gridZ);
        float dstY = Mathf.Abs(a.worldPosition.y - b.worldPosition.y);

        if (dstX > dstZ)
            return 14f * dstZ + 10f * (dstX - dstZ) + dstY;
        return 14f * dstX + 10f * (dstZ - dstX) + dstY;
    }
    private float CalcularLongitudRuta(List<Vector3> path)
    {
        float length = 0f;
        for (int i = 0; i < path.Count - 1; i++)
        {
            length += Vector3.Distance(path[i], path[i + 1]);
        }
        return length;
    }


    // Limita una posición a los límites del terreno
    private Vector3 LimitarPosicionAlTerreno(Vector3 position)
    {
        if (terrain == null)
        {
            Debug.LogError("El terreno no está inicializado en LimitarPosicionAlTerreno.");
            return position;
        }

        position.x = Mathf.Clamp(position.x, 0, terrainSize.x);
        position.z = Mathf.Clamp(position.z, 0, terrainSize.z);
        position.y = terrain.SampleHeight(position) + gridHeightOffset;
        return position;
    }

    
    // Dibuja el camino en la escena para visualización
    private void DibujarCamino(List<Vector3> path)
    {
        if (path == null || path.Count < 2)
            return;

        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i + 1], pathColor, pathDuration);
        }

        Vector3 textPosition = (path[0] + path[path.Count - 1]) / 2f + Vector3.up * 2f;
        Debug.DrawLine(textPosition, textPosition + Vector3.up, Color.white, pathDuration);
        Debug.DrawLine(textPosition + Vector3.up, textPosition + Vector3.up + Vector3.right * 2f, Color.white, pathDuration);
        Debug.DrawLine(textPosition + Vector3.up + Vector3.right * 2f, textPosition + Vector3.right * 2f, Color.white, pathDuration);
        Debug.DrawLine(textPosition + Vector3.right * 2f, textPosition, Color.white, pathDuration);
    }


    // Dibuja gizmos para visualizar la cuadrícula en el editor
    private void OnDrawGizmos()
    {
        if (grid != null)
        {
            foreach (Node node in grid)
            {
                // Crear un color azul con 10% de transparencia
                Gizmos.color = node.walkable ? new Color(0, 0, 1, 0.1f) : Color.red;
                Gizmos.DrawCube(node.worldPosition, Vector3.one * (nodeRadius - 0.1f));
            }
        }
    }
}