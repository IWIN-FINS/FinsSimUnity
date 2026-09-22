using UnityEngine;
using UnityEngine.InputSystem;
public class Control : MonoBehaviour
{
    public Rigidbody rb;
    public float moveSpeed = 5f;

    void FixedUpdate()
    {
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(horizontal, 0, vertical).normalized * moveSpeed;
        rb.linearVelocity = movement;
    }
}
