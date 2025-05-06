using UnityEngine;
using RosSharp.RosBridgeClient;
using Float32MultiArray = RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray;

public class CameraTargetSubscriber : UnitySubscriber<Float32MultiArray>
{
    private bool _receivedTarget = false; //Флаг, чи взагалі ми вже отримали хоч одне повідомлення з координатами.
    public Camera cameraToControl; //Посилання на компонент Camera, яку ми будемо рухати.
    public float moveSpeed = 2.0f; //Швидкість інтерполяції (смузування) руху камер
    public float rotationSpeed = 2f;
    // Нормалізований вектор напрямку променя з камери до цілі,
    // за яким будемо рухати та повертати камеру
    private Vector3 _dir;
    
    protected override void Start()
    {
        base.Start();  // підключаємося до RosBridge
        //Перевіряємо, чи в інспекторі призначена камера:
        //якщо ні — підставляємо за замовчуванням головну камеру сцени
        if (cameraToControl == null) 
            cameraToControl = Camera.main;
    }
    
    protected override void ReceiveMessage(Float32MultiArray message)
    {
        if (message.data.Length >= 2)
        {
            if (message.data.Length >= 2)
            {
                float xNorm = message.data[0];
                float yNorm = message.data[1];
                Ray ray = cameraToControl.ViewportPointToRay(
                    new Vector3(xNorm, yNorm, 0f)
                );
                _dir = ray.direction.normalized;
                _receivedTarget = true;
            }
        }
    }
    
    private void Update()
    {
        if (!_receivedTarget) return;

        Transform camT = cameraToControl.transform;

        //Поворот
        Quaternion targetRot = Quaternion.LookRotation(_dir, Vector3.up);
        camT.rotation = Quaternion.Slerp(
            camT.rotation, targetRot, Time.deltaTime * rotationSpeed
        );

        //Рух уперед у напрямку центру кадру
        camT.position += _dir * moveSpeed * Time.deltaTime;
    }
}
