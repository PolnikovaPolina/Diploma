using UnityEngine;

public class ZalaTrajectory : MonoBehaviour
{
    public float radius = 500f;
    public float speed = 30f;
    private float angle = 0f;
    private Vector3 center;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
                center = transform.position;

                // Measure the size of the object
        Renderer renderer = GetComponent<Renderer>();
        if (renderer != null)
        {
            Vector3 size = renderer.bounds.size;
            Debug.Log("Object size: " + size);
        }
        else
        {
            Debug.LogWarning("No Renderer component found on this object.");
        }

    }

    // Update is called once per frame
    void Update()
    {
           // Calculate the angle increment based on speed and radius
        angle -= speed * Time.deltaTime / radius;

        // Calculate the new position
        float x = center.x + Mathf.Cos(angle) * radius;
        float z = center.z + Mathf.Sin(angle) * radius;
        Vector3 newPosition = new Vector3(x, transform.position.y, z);

        // Update the position of the object
        transform.position = newPosition;

       
    // Calculate the direction of movement
    Vector3 direction = newPosition - center;
      // the second argument, upwards, defaults to Vector3.up
    Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);

    // Combine with a -90 degree rotation around the x-axis
    Quaternion finalRotation = rotation * Quaternion.Euler(-90, 0, 0);

    transform.rotation = finalRotation;

    // Set the rotation to look in the direction of movement
    // if (direction != Vector3.zero)
    // {
    //     Quaternion rotation = Quaternion.LookRotation(direction);
    //     transform.rotation = Quaternion.Euler(-90, rotation.eulerAngles.y, 0);
    // }
    }
}
