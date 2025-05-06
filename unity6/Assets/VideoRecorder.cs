using UnityEngine;
#if UNITY_EDITOR
using UnityEditor.Recorder;           // Простір імен плагіна Unity Recorder
using UnityEditor.Recorder.Input;     // Налаштування джерела кадру (Game View тощо)
using UnityEditor.Recorder.Encoder;
#endif

public class VideoRecorder : MonoBehaviour
{
    #if UNITY_EDITOR //певні рядки коду включалися лише тоді, коли ви знаходитесь в самому редакторі Unity, а не в зібраному (build-і) додатку.
    private RecorderController _controller;  // Об’єкт, який керує записом
    #endif

    void Awake()
    {
        #if UNITY_EDITOR
        // 1) Створюємо набір налаштувань контролера
        var settings = ScriptableObject.CreateInstance<RecorderControllerSettings>();

        // 2) Створюємо MovieRecorderSettings
        var movie = ScriptableObject.CreateInstance<MovieRecorderSettings>();
        movie.name            = "HitVideo";
        movie.Enabled         = true;
        movie.FrameRate       = 30f;
        
        // 3) Замість MP4EncoderSettings — робимо CoreEncoderSettings
        var coreEncoder = new CoreEncoderSettings();
        coreEncoder.Codec       = CoreEncoderSettings.OutputCodec.H264_MP4;       // H.264 у MP4-контейнері
        coreEncoder.EncodingQuality = CoreEncoderSettings.VideoEncodingQuality.High;  // Висока якість
        // Додатково за потреби:
        // coreEncoder.TargetBitRate = 10f;  // Mbps
        movie.EncoderSettings   = coreEncoder;

        // 4) Вказуємо джерело кадру
        movie.ImageInputSettings = new GameViewInputSettings();

        // 5) Додаємо записувач у контролер
        settings.AddRecorderSettings(movie);
        settings.SetRecordModeToManual();

        // 6) Ініціалізуємо контролер
        _controller = new RecorderController(settings);
        #endif
    }

    // Виклик із іншого скрипта — почати запис
    public void StartRecording()
    {
        #if UNITY_EDITOR
        _controller.PrepareRecording();
        _controller.StartRecording();
        #endif
    }

    // Виклик із іншого скрипта — зупинити запис і зберегти файл
    public void StopRecording()
    {
        #if UNITY_EDITOR
        _controller.StopRecording();
        #endif
    }
}