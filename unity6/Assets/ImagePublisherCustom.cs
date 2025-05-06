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


namespace RosSharp.RosBridgeClient
{
    public class ImagePublisher : UnityPublisher<MessageTypes.Sensor.CompressedImage>
    {
        public Camera ImageCamera;
        public string FrameId = "Camera";
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        [Range(0, 100)]
        public int qualityLevel = 50;

        private MessageTypes.Sensor.CompressedImage message;
        private Texture2D texture2D;
        private Rect rect;

        protected override void Start()
        {
            base.Start();
            InitializeGameObject();
            InitializeMessage();
            RenderPipelineManager.endCameraRendering += UpdateImage;
            // Camera.onPostRender += Test;
            // Camera.onPostRender += UpdateImage;
             // Output a debug message to the console
            //  Debug.Log("ImagePublisherCustom Start method called");

        }
        void OnDestroy()
        {
            RenderPipelineManager.endCameraRendering -= UpdateImage;
        }
        private void UpdateImage(ScriptableRenderContext context, Camera _camera)
        {
            // Debug.Log("ImagePublisherCustom UpdateImage method called");
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
            message = new MessageTypes.Sensor.CompressedImage();
            message.header = new MessageTypes.Std.Header(); 
            message.header.frame_id = FrameId;
            message.format = "jpeg";
        }

        private void UpdateMessage()
        {
            double t = Time.realtimeSinceStartupAsDouble;
            message.header.frame_id = t.ToString("F5"); // з точністю до 5 знаків
            message.header.Update();
            texture2D.ReadPixels(rect, 0, 0);
            message.data = texture2D.EncodeToJPG(qualityLevel);
            Publish(message);
        }

    }
}
