using UnityEngine;
using RosSharp.RosBridgeClient; // для UnityPublisher
using RosSharp.RosBridgeClient.MessageTypes.Std; // для Float32MultiArray передаємо масив чисел типу float


public class PosePixelObject : UnityPublisher<Float32MultiArray>
{
    private Float32MultiArray _message;
    public Camera _mainCamera;
    public GameObject target;

    protected override void Start()
    {
        Application.runInBackground = true; // дозволяємо Unity працювати у фоні
        base.Start();
        if (_mainCamera == null)
            _mainCamera = Camera.main;
        _message = new Float32MultiArray();
    }

    private void Update()
    {
        if (!ReferenceEquals(target, null)  && !ReferenceEquals(_mainCamera, null))
        {
            Vector3 screenPos = _mainCamera.WorldToScreenPoint(target.transform.position);

            float uNorm = screenPos.x / _mainCamera.pixelWidth;
            float vNorm = 1f - (screenPos.y / _mainCamera.pixelHeight); 
            float t = (float)System.Math.Round(Time.realtimeSinceStartupAsDouble, 5); // з точністю до 5 знаків

            _message.data = new float[] { t, uNorm, vNorm };

            Publish(_message);
        }
    }
}
