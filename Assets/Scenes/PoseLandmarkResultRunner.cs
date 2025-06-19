using System;
using System.Collections;
using Mediapipe.Tasks.Vision.PoseLandmarker;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Serialization;

namespace Mediapipe.Unity.Sample.PoseLandmarkDetection
{
  public class PoseLandmarkResultRunner : VisionTaskApiRunner<PoseLandmarker>
  {
    private Experimental.TextureFramePool _textureFramePool;
    private readonly LandmarkGizmo _landmarkGizmo = new();
    [SerializeField] private LandmarkRig landmarkRig;
    [SerializeField] private PoseLandmarkerResultAnnotationController poseLandmarkerResultAnnotationController;

    public override void Stop()
    {
      base.Stop();
      _textureFramePool?.Dispose();
      _textureFramePool = null;
    }

    protected override IEnumerator Run()
    {
      const string poseModelPath = "pose_landmarker_full.bytes";
      yield return AssetLoader.PrepareAssetAsync(poseModelPath);

      var options = new PoseLandmarkerOptions(
        new Tasks.Core.BaseOptions(modelAssetPath: poseModelPath),
        runningMode: Tasks.Vision.Core.RunningMode.VIDEO
      );
      taskApi = PoseLandmarker.CreateFromOptions(options, GpuManager.GpuResources);
      var imageSource = ImageSourceProvider.ImageSource;

      yield return imageSource.Play();
      if (!imageSource.isPrepared)
      {
        Logger.LogError(TAG, "Failed to start ImageSource, exiting...");
        yield break;
      }

      // Use RGBA32 as the input format.
      _textureFramePool = new Experimental.TextureFramePool(imageSource.textureWidth, imageSource.textureHeight);
      // NOTE: The screen will be resized later, keeping the aspect ratio.
      screen.Initialize(imageSource);
      SetupAnnotationController(poseLandmarkerResultAnnotationController, imageSource);
      poseLandmarkerResultAnnotationController.InitScreen(imageSource.textureWidth, imageSource.textureHeight);

      var transformationOptions = imageSource.GetTransformationOptions();
      var flipHorizontally = transformationOptions.flipHorizontally;
      var flipVertically = transformationOptions.flipVertically;
      var imageProcessingOptions = new Tasks.Vision.Core.ImageProcessingOptions(rotationDegrees: 0);

      AsyncGPUReadbackRequest req = default;
      var waitUntilReqDone = new WaitUntil(() => req.done);
      var result = PoseLandmarkerResult.Alloc(options.numPoses, options.outputSegmentationMasks);

      while (true)
      {
        if (!_textureFramePool.TryGetTextureFrame(out var textureFrame))
        {
          yield return new WaitForEndOfFrame();
          continue;
        }

        // Build the input Image
        req = textureFrame.ReadTextureAsync(imageSource.GetCurrentTexture(), flipHorizontally, flipVertically);
        yield return waitUntilReqDone;

        if (req.hasError) continue;
        var image = textureFrame.BuildCPUImage();
        textureFrame.Release();

        if (taskApi.TryDetectForVideo(image, GetCurrentTimestampMillisec(), imageProcessingOptions, ref result))
        {
          poseLandmarkerResultAnnotationController.DrawNow(result);
          _landmarkGizmo.SetResult(result);
          landmarkRig.SetPose(result);
        }
        else
        {
          poseLandmarkerResultAnnotationController.DrawNow(default);
        } 
        DisposeAllMasks(result);
      }
    }

    private void DisposeAllMasks(PoseLandmarkerResult result)
    {
      if (result.segmentationMasks != null)
      {
        foreach (var mask in result.segmentationMasks)
        {
          mask.Dispose();
        }
      }
    }

    private void OnDrawGizmos()
    {
      _landmarkGizmo.DrawLandmarks();
    }
  }
}
