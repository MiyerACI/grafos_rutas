using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CharacterController : MonoBehaviour
{
    // Referencias y configuración
    public AStarPathfinding pathfinding;       // Referencia al componente de pathfinding
    public float moveSpeed = 5f;               // Velocidad de movimiento del personaje
    public float rotationSpeed = 5f;           // Velocidad de rotación del personaje
    public float pathUpdateInterval = 1f;      // Intervalo de actualización del camino
    public float stoppingDistance = 0.1f;      // Distancia a la que el personaje se detiene al llegar a un waypoint

    // Variables privadas
    private List<Vector3> path;                // Camino actual
    private int currentWaypointIndex = 0;      // Índice del waypoint actual
    private Animator animator;                 // Referencia al componente Animator

    // Inicialización
    private void Start()
    {
        InicializarComponentes();
        StartCoroutine(ActualizarCaminoRutina());
    }

    // Inicializa los componentes necesarios
    private void InicializarComponentes()
    {
        if (pathfinding == null)
        {
            Debug.LogError("El objeto Pathfinding no está asignado. Asegúrate de arrastrar el objeto Pathfinder en el Inspector.");
            return;
        }

        animator = GetComponent<Animator>();
        if (animator == null)
        {
            Debug.LogWarning("No se encontró un componente Animator en el personaje.");
        }
    }

    // Corrutina para actualizar el camino periódicamente
    private IEnumerator ActualizarCaminoRutina()
    {
        while (true)
        {
            ActualizarCamino();
            yield return new WaitForSeconds(pathUpdateInterval);
        }
    }

    // Actualiza el camino usando el componente de pathfinding
    private void ActualizarCamino()
    {
        if (pathfinding == null || pathfinding.targetPoint == null)
        {
            Debug.LogError("Pathfinding o punto objetivo no está configurado.");
            return;
        }

        path = pathfinding.EncontrarCamino(transform.position);
        if (path != null && path.Count > 0)
        {
            currentWaypointIndex = 0;
            Debug.Log($"Nuevo camino encontrado. Longitud: {path.Count} waypoints");
        }
        else
        {
            Debug.LogWarning("No se pudo encontrar un camino válido. El personaje intentará moverse directamente hacia el objetivo.");
            path = new List<Vector3> { pathfinding.targetPoint.position };
        }
    }

    // Actualización por frame
    private void Update()
    {
        if (path != null && path.Count > 0)
        {
            MoverSiguiendoCamino();
        }
        else
        {
            DetenerMovimiento();
        }
    }

    // Mueve al personaje a lo largo del camino
    private void MoverSiguiendoCamino()
    {
        Vector3 targetPosition = path[currentWaypointIndex];
        Vector3 direction = (targetPosition - transform.position).normalized;
        direction.y = 0; // Mantener el movimiento horizontal

        // Mover hacia el objetivo
        transform.position = Vector3.MoveTowards(transform.position, targetPosition, moveSpeed * Time.deltaTime);

        // Rotar hacia la dirección del movimiento
        if (direction != Vector3.zero)
        {
            Quaternion targetRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }

        // Actualizar animación
        if (animator != null)
        {
            animator.SetBool("IsWalking", true);
        }

        // Verificar si hemos llegado al waypoint actual
        if (Vector3.Distance(transform.position, targetPosition) < stoppingDistance)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex >= path.Count)
            {
                DetenerMovimiento();
            }
        }
    }

    // Detiene el movimiento del personaje
    private void DetenerMovimiento()
    {
        if (animator != null)
        {
            animator.SetBool("IsWalking", false);
        }
    }
}