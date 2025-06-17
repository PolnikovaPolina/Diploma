/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Adjustments to new Publication Timing and Execution Framework 
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using UnityEngine;
using UnityEngine.Rendering;
using RosSharp.RosBridgeClient.MessageTypes.Std; // для Float32MultiArray
using RosSharp.RosBridgeClient.MessageTypes.Sensor; // для CompressedImage
using RosSharp.RosBridgeClient.MessageTypes.MyInterface;

namespace RosSharp.RosBridgeClient
{
    public class ImagePublisher : UnityPublisher<CameraStamped>
    {
        public Camera ImageCamera;
        public string FrameId = "Camera";
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        [Range(0, 100)]
        public int qualityLevel = 50;
        public GameObject target;
        private CameraStamped message;
        
        private Texture2D texture2D;
        private Rect rect;

        protected override void Start()
        {
            Application.runInBackground = true; // дозволяємо Unity працювати у фоні
            base.Start();
            InitializeGameObject();
            InitializeMessage();
            // Camera.onPostRender += Test;
            // Camera.onPostRender += UpdateImage;
            // Output a debug message to the console
            // Debug.Log("ImagePublisherCustom Start method called");
            RenderPipelineManager.endCameraRendering += UpdateImage;

        }
        void OnDestroy()
        {
            RenderPipelineManager.endCameraRendering -= UpdateImage;
        }
        
        private void UpdateImage(ScriptableRenderContext context, Camera _camera)
        {
            //Debug.Log($"[ImagePublisher] UpdateImage called for camera {_camera.name}");
            if (texture2D != null && _camera == this.ImageCamera)
                UpdateMessage();
        }

        private void InitializeGameObject()
        {
            texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
            rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
            ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        }

        private void InitializeMessage()
        {
            message = new CameraStamped
            {
                header = new Header { frame_id = FrameId },
                image  = new CompressedImage { format = "jpeg" },
                status = new Float32MultiArray()
            };
        }

        private void UpdateMessage()
        {
            double t = Time.realtimeSinceStartupAsDouble;
            message.header.frame_id = t.ToString("F5"); // з точністю до 5 знаків
            message.header.Update();
            texture2D.ReadPixels(rect, 0, 0);
            message.image.data = texture2D.EncodeToJPG(qualityLevel);
            Vector3 camP   = ImageCamera.transform.position;
            Quaternion camQ = ImageCamera.transform.rotation;
            Vector3 tgtP   = target.transform.position;
            // field Of View для обчислення фокусів камери
            float verticalFovDeg = ImageCamera.fieldOfView;
            float verticalFovRad = verticalFovDeg * Mathf.Deg2Rad;
            float aspect = (float)resolutionWidth / resolutionHeight;
            float horizontalFovRad = 2f * Mathf.Atan(Mathf.Tan(verticalFovRad / 2f) * aspect);
            float horizontalFovDeg = horizontalFovRad * Mathf.Rad2Deg;
            Debug.Log($"Vertical FOV: {verticalFovDeg:F2}°, Horizontal FOV: {horizontalFovDeg:F2}°");
            // Публікація
            message.status.data = new float[]
            {
                verticalFovDeg, horizontalFovDeg,
                camP.x, camP.y, camP.z,
                camQ.x, camQ.y, camQ.z, camQ.w,
                tgtP.x, tgtP.y, tgtP.z
            };
            Publish(message);
        }
    }
}
