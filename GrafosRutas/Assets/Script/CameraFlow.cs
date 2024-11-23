using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;  // El objetivo que la cámara seguirá (tu personaje)
    public Vector3 offset;    // Desplazamiento de la cámara respecto al objetivo
    public float smoothSpeed = 0.125f; // Velocidad de suavizado

    private void LateUpdate()
    {
        if (target != null)
        {
            // Calcula la posición deseada de la cámara
            Vector3 desiredPosition = target.position + offset;
            // Suaviza la posición de la cámara
            Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
            transform.position = smoothedPosition;

            // Asegúrate de que la cámara mire al personaje
            transform.LookAt(target);
        }
    }
}