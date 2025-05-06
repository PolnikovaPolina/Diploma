using UnityEngine;
using UnityEditor.Recorder;           // Простір імен плагіна Unity Recorder
using UnityEditor.Recorder.Input;     // Налаштування джерела кадру (Game View тощо)
using UnityEditor.Recorder.Encoder;

public class VideoRecorder : MonoBehaviour
{
   private RecorderController _controller;  // Об’єкт, який керує записом
    
    void Awake()
    {
        // 1) Створюємо набір налаштувань контролера
        var settings = ScriptableObject.CreateInstance<RecorderControllerSettings>();

        // 2) Створюємо MovieRecorderSettings
        var movie = ScriptableObject.CreateInstance<MovieRecorderSettings>();
        movie.name = "HitVideo";
        movie.Enabled = true;
        movie.FrameRate = 30f;
        
        // 3) Замість MP4EncoderSettings — робимо CoreEncoderSettings
        var coreEncoder = new CoreEncoderSettings();
        coreEncoder.Codec = CoreEncoderSettings.OutputCodec.MP4;    // H.264 у MP4-контейнері
        coreEncoder.EncodingQuality = CoreEncoderSettings.VideoEncodingQuality.High;  // Висока якість
        movie.EncoderSettings   = coreEncoder;

        // 4) Вказуємо джерело кадру
        movie.ImageInputSettings = new GameViewInputSettings();

        // 5) Додаємо записувач у контролер
        settings.AddRecorderSettings(movie);
        settings.SetRecordModeToManual();

        // 6) Ініціалізуємо контролер
        _controller = new RecorderController(settings);
    }

    // Виклик із іншого скрипта — почати запис
    public void StartRecording()
    {
        _controller.PrepareRecording();
        _controller.StartRecording();
    }

    // Виклик із іншого скрипта — зупинити запис і зберегти файл
    public void StopRecording()
    {
        _controller.StopRecording();
    }
}