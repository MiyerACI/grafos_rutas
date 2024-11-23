using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;  // El objetivo que la c�mara seguir� (tu personaje)
    public Vector3 offset;    // Desplazamiento de la c�mara respecto al objetivo
    public float smoothSpeed = 0.125f; // Velocidad de suavizado

    private void LateUpdate()
    {
        if (target != null)
        {
            // Calcula la posici�n deseada de la c�mara
            Vector3 desiredPosition = target.position + offset;
            // Suaviza la posici�n de la c�mara
            Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
            transform.position = smoothedPosition;

            // Aseg�rate de que la c�mara mire al personaje
            transform.LookAt(target);
        }
    }
}