using RosSharp.RosBridgeClient; // для UnityPublisher
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

public class PoseObject : UnityPublisher<Pose>
{
    public UnityEngine.GameObject target;
    private Pose _message;

    protected override void Start()
    {
        base.Start();
        _message = new Pose
        {
            position    = new Point(),
            orientation = new Quaternion()
        };
    }

    private void Update()
    {
        if (!ReferenceEquals(target, null))
        {
            UnityEngine.Vector3 pos = target.transform.position;
            UnityEngine.Quaternion rot = target.transform.rotation;
            
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

