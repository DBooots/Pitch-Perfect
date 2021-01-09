using System;
using UnityEngine;

namespace PitchPerfect
{
    public static class PropControlMath
    {
        public static bool GetDeployAngle(Vector3 inflowDir, Vector3 pitchAxis, Vector3 liftVector_0, float targetDot, out float deployAngle, bool minus = false)
        {
            if (targetDot < -1 || targetDot > 1)
                throw new ArgumentOutOfRangeException("targetDot");

            float a = Vector3.Dot(inflowDir, pitchAxis) * Vector3.Dot(pitchAxis, liftVector_0);
            float b = Vector3.Dot(inflowDir, liftVector_0);
            float c = Vector3.Dot(inflowDir, Vector3.Cross(pitchAxis, liftVector_0));

            float ab = a - b;
            float da = targetDot - a;
            float ab2 = ab * ab;
            float da2 = da * da;
            float c2 = c * c;
            
            if (4 * (ab2 + c2) * (da2 - c2) > da2 * ab2)
            {
                deployAngle = float.NaN;
                return false;
            }

            deployAngle = Mathf.Acos((-da * ab + (minus ? -1 : 1) * Mathf.Sqrt(da2 * ab2 - 4 * (ab2 + c2) * (da2 - c2))) / (2 * (ab2 + c2))) * Mathf.Rad2Deg;
            if (deployAngle > 90)
                deployAngle -= 180;
            return true;
        }

        public static float ThrustFraction(float pitchAngle, Vector3 rotationAxis, Vector3 pitchAxis, Vector3 liftVector_0)
        {
            return Vector3.Project(Quaternion.AngleAxis(pitchAngle, pitchAxis) * liftVector_0, rotationAxis).magnitude;
        }

        public static float LiftOverDragFraction(float pitchAngle, Vector3 rotationAxis, Vector3 pitchAxis, Vector3 liftVector_0, Vector3 actionPoint)
        {
            Vector3 LiftVector = Quaternion.AngleAxis(pitchAngle, pitchAxis) * liftVector_0;
            return Mathf.Sqrt(Vector3.Project(LiftVector, rotationAxis).sqrMagnitude / Vector3.Project(Vector3.Cross(LiftVector, actionPoint), rotationAxis).sqrMagnitude);
        }
    }
}
