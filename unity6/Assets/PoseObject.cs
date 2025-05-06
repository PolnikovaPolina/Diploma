using UnityEngine;
using RosSharp.RosBridgeClient; // для UnityPublisher
using RosPose = RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose;
using RosSharp.RosBridgeClient.MessageTypes.Std; // для Float32MultiArray передаємо масив чисел типу float

public class PoseObject : UnityPublisher<RosPose>
{
    public GameObject target;
    private RosPose _message;

    protected override void Start()
    {
        base.Start();
        _message = new RosPose();
    }

    private void Update()
    {
        if (!ReferenceEquals(target, null))
        {
            Vector3 pos = target.transform.position;
            Quaternion rot = target.transform.rotation;
            
            _message.position.x = pos.x;
            _message.position.y = pos.y;
            _message.position.z = pos.z;

            _message.orientation.x = rot.x;
            _message.orientation.y = rot.y;
            _message.orientation.z = rot.z;
            _message.orientation.w = rot.w;

            Publish(_message);
        }
    }
}

