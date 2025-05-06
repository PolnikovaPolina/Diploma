using UnityEngine;
using RosSharp.RosBridgeClient; // для UnityPublisher
using PixelPose = RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray; // для Float32MultiArray передаємо масив чисел типу float


public class PosePixelObject : UnityPublisher<PixelPose>
{
    private PixelPose _message;
    private Camera _mainCamera;
    public GameObject target;

    protected override void Start()
    {
        base.Start();
        _mainCamera = Camera.main; // викликається лише один раз
        _message = new PixelPose();
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
