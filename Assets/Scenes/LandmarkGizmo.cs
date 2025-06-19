using System.Collections.Generic;
using UnityEngine;
using Mediapipe.Tasks.Vision.PoseLandmarker;
using Mediapipe.Tasks.Components.Containers;

public class LandmarkGizmo
{
    private PoseLandmarkerResult _result;
    private readonly Vector3 _offset = new Vector3(1, 1, 1);

    public void SetResult(PoseLandmarkerResult result)
    {
        this._result = result;
    }
    
    public void DrawLandmarks()
    {
        if (_result.poseLandmarks == null) return;
        var poseWorldLandmark = _result.poseWorldLandmarks[0];
        
        // Set the color with custom alpha.
        const float radius = 0.015f;
        const float alpha = 0.5f;
        

        foreach (var landmark in poseWorldLandmark.landmarks)
        {
            var radiusWeight = 1.0f;
            if (landmark.presence != null) radiusWeight = (float)landmark.presence;
            Gizmos.DrawSphere(LandmarkToVector(landmark), radius * radiusWeight);

            var colorWeight = 1.0f;
            if (landmark.visibility != null) colorWeight = (float)landmark.visibility;
            Gizmos.color = new Color(1f - colorWeight, 0f,colorWeight, alpha); // Red with custom alpha
        }
        
        var pairs = new List<(int, int)>
        {
            (12, 11), (12, 24), (11, 23), (24, 23),
            (24, 26), (23, 25), (26, 28), (25, 27),
            (28, 32), (27, 31), (28, 30), (27, 29),
            (32, 30), (31, 29), (14, 12), (11, 13),
            (14, 16), (13, 15), (0, 5), (0, 1),
        };
      
        foreach (var (i1, i2) in pairs)
        {
            var from = LandmarkToVector(poseWorldLandmark.landmarks[i1]);
            var to = LandmarkToVector(poseWorldLandmark.landmarks[i2]);
            Gizmos.DrawLine(from, to);
        }
    }
    
    private Vector3 LandmarkToVector(Landmark landmark)
    {
        var x = landmark.x;
        var y = -landmark.y;
        var z = landmark.z;
        return new Vector3(x, y, z) + _offset;
    }
}
