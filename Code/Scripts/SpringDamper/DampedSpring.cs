using System;
using UnityEngine;

namespace BoschingMachine
{
    [Serializable]
    public sealed class DampedSpring
    {
        public SpringDamperSettings settings;
        private SpringDamperState x;
        private SpringDamperState y;
        private SpringDamperState z;

        public Vector3 PositionV { get; set; }
        public Vector3 VelocityV { get; set; }

        public float PositionF => PositionV.x;
        public float VelocityF => PositionV.x;
        
        public ProcessOperation Operation { get; set; } = DampedSpring.ProcessLinear;

        public DampedSpring() : this(1.0f, 0.5f, 2.0f) { }
        public DampedSpring(float f, float z, float r) : this(Vector3.zero, f, z, r) { }
        public DampedSpring(Vector3 lastTarget, float f, float z, float r)
        {
            settings = new SpringDamperSettings(f, z, r);
            x = new SpringDamperState(lastTarget.x);
            y = new SpringDamperState(lastTarget.y);
            this.z = new SpringDamperState(lastTarget.z);
        }

        public void Update(float target, float dt) => Update(target, null, dt);
        public void Update(float target, float? targetVelocity, float dt) => Update(Vector3.right * target, targetVelocity.HasValue ? Vector3.right * targetVelocity.Value : null, dt);
        public void Update(Vector3 target, float dt) => Update(target, null, dt);
        public void Update(Vector3 target, Vector3? targetVelocity, float dt)
        {
            float Func(Func<Vector3, float> f, SpringDamperState state)
            {
                return Operation(f(PositionV), f(VelocityV), f(target), targetVelocity.HasValue ? f(targetVelocity.Value) : null, dt, settings, state);
            }

            var fx = Func(v => v.x, x);
            var fy = Func(v => v.y, y);
            var fz = Func(v => v.z, z);

            PositionV += VelocityV * dt;
            VelocityV += new Vector3(fx, fy, fz) * dt;
        }
        
        public delegate float ProcessOperation(float position, float velocity, float targetPosition, float? targetVelocity, float dt, SpringDamperSettings settings, SpringDamperState state);

        public static float ProcessRotation(float position, float velocity, float targetPosition, float? targetVelocity, float dt, SpringDamperSettings settings, SpringDamperState state)
        {
            var altTarget = targetPosition < 180.0f ? targetPosition + 360.0f : targetPosition - 360.0f;
            if (Mathf.Abs(targetPosition - position) > Mathf.Abs(altTarget - position)) targetPosition = altTarget;

            targetVelocity ??= Mathf.DeltaAngle(targetPosition, state.lastTargetPosition) / dt;

            return ProcessLinear(position, velocity, targetPosition, targetVelocity, dt, settings, state);
        }

        public static float ProcessLinear(float position, float velocity, float targetPosition, float? targetVelocity, float dt, SpringDamperSettings settings, SpringDamperState state)
        {
            targetVelocity ??= (targetPosition - state.lastTargetPosition) / dt;

            var w = 2.0f * Mathf.PI * settings.F;
            var z = settings.Z;
            var d = w * Mathf.Sqrt(Mathf.Abs(z * z - 1));

            float k1Stable, k2Stable;
            if (w * dt < z)
            {
                k1Stable = settings.k1;
                k2Stable = Mathf.Max(settings.k2, dt * dt / 2.0f + dt * settings.k1 / 2.0f, dt * settings.k1);
            }
            else
            {
                var t1 = Mathf.Exp(-z * w * dt);
                var alpha = 2.0f * t1 * (z <= 1.0f ? Mathf.Cos(dt * d) : MathF.Cosh(dt * d));
                var beta = t1 * t1;
                var t2 = dt / (1.0f + beta - alpha);
                k1Stable = (1.0f - beta) * t2;
                k2Stable = dt * t2;
            }

            state.lastTargetPosition = targetPosition;

            return (dt * (targetPosition + settings.k3 * targetVelocity.Value - position - k1Stable * velocity) / k2Stable) / dt;
        }

        public static void GetAbstracts(in float k1, in float k2, in float k3, out float f, out float z, out float r)
        {
            f = GetF(k1, k2, k3);
            z = GetZ(k1, k2, k3);
            r = GetR(k1, k2, k3);
        }

        public static void CalculateAbstracts(in float f, in float z, in float r, out float k1, out float k2, out float k3)
        {
            k1 = z / (Mathf.PI * f);
            k2 = 1.0f / Mathf.Pow(2.0f * Mathf.PI * f, 2);
            k3 = (r * z) / (2.0f * Mathf.PI * f);
        }

        public static float GetF(float k1, float k2, float k3) => 1.0f / (2.0f * Mathf.PI * Mathf.Sqrt(k2));
        public static float GetZ(float k1, float k2, float k3) => k1 / (2.0f * Mathf.Sqrt(k2));
        public static float GetR(float k1, float k2, float k3) => (2.0f * k3) / k1;
    }

    [System.Serializable]
    public class SpringDamperSettings
    {
        public float k1;
        public float k2;
        public float k3;

        public float F
        {
            get => DampedSpring.GetF(k1, k2, k3);
            set => SetAbstracts(value, Z, R);
        }

        public float Z
        {
            get => DampedSpring.GetZ(k1, k2, k3);
            set => SetAbstracts(Z, value, R);
        }

        public float R
        {
            get => DampedSpring.GetR(k1, k2, k3);
            set => SetAbstracts(F, Z, value);
        }

        public SpringDamperSettings()
        {
            SetAbstracts(1.0f, 0.5f, 2.0f);
        }

        public SpringDamperSettings(float f, float z, float r)
        {
            SetAbstracts(f, z, r);
        }

        public SpringDamperSettings SetAbstracts(float f, float z, float r)
        {
            DampedSpring.CalculateAbstracts(f, z, r, out k1, out k2, out k3);
            return this;
        }
    }

    [System.Serializable]
    public class SpringDamperState
    {
        public float lastTargetPosition;

        public SpringDamperState(float lastTargetPosition = 0.0f)
        {
            this.lastTargetPosition = lastTargetPosition;
        }
    }
}