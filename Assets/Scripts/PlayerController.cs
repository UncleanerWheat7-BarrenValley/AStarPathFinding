using System.Globalization;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    public float speed = 5f;

    void Update()
    {
        float moveX = Input.GetAxis("Horizontal");
        float moveZ = Input.GetAxis("Vertical");

        // Move the player based on input.
        transform.Translate(new Vector3(moveX, 0, moveZ) * speed * Time.deltaTime);
    }
}
