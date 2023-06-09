using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace BoschingMachine.Editor
{
    [CustomPropertyDrawer(typeof(SpringDamperSettings))]
    public sealed class DampedSpringsPropertyDrawer : PropertyDrawer
    {
        const float lnHeight = 18.0f;

        [SerializeField] bool foldoutState;

        bool initalized;
        float f;
        float z;
        float r;
        float previewDuration = 2.0f;

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            return foldoutState ? lnHeight * 7.0f + 100.0f : lnHeight;
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);

            var fieldIndex = 0;
            System.Func<Rect> getFieldRect = () =>
            {
                return new Rect(position.x, position.y + fieldIndex++ * lnHeight, position.width, lnHeight);
            };

            if (foldoutState = EditorGUI.Foldout(getFieldRect(), foldoutState, label))
            {
                EditorGUI.indentLevel++;

                var k1Prop = property.FindPropertyRelative("k1");
                var k2Prop = property.FindPropertyRelative("k2");
                var k3Prop = property.FindPropertyRelative("k3");

                var k1 = k1Prop.floatValue;
                var k2 = k2Prop.floatValue;
                var k3 = k3Prop.floatValue;

                if (!initalized)
                {
                    DampedSpring.GetAbstracts(k1, k2, k3, out f, out z, out r);
                    initalized = true;
                }

                f = EditorGUI.FloatField(getFieldRect(), "F", f);
                z = EditorGUI.FloatField(getFieldRect(), "Z", z);
                r = EditorGUI.FloatField(getFieldRect(), "R", r);
                fieldIndex++;
                previewDuration = Mathf.Max(0.0f, EditorGUI.FloatField(getFieldRect(), "Preview Duration", previewDuration));

                DampedSpring.CalculateAbstracts(f, z, r, out k1, out k2, out k3);

                k1Prop.floatValue = k1;
                k2Prop.floatValue = k2;
                k3Prop.floatValue = k3;

                DrawGraph(getFieldRect(), f, z, r);
            }
        }

        private void DrawGraph(Rect baseRect, float f, float z, float r)
        {
            baseRect.height = 100.0f;
            baseRect.width -= lnHeight;
            EditorGUI.DrawRect(baseRect, ColorLib.Gray(28));

            var data = GetGraphData(f, z, r, out var min, out var max, previewDuration);
            System.Func<Vector2, Vector2> fromGraphPos = v => new Vector2(baseRect.x + v.x * baseRect.width, baseRect.y + (1.0f - Mathf.InverseLerp(min, max, v.y)) * baseRect.height);

            Handles.color = ColorLib.Gray(40);
            Handles.DrawLine(fromGraphPos(Vector2.zero), fromGraphPos(Vector2.right));
            Handles.DrawLine(fromGraphPos(Vector2.up), fromGraphPos(Vector2.one));

            Handles.Label(fromGraphPos(Vector2.zero), "0.0");
            Handles.Label(fromGraphPos(Vector2.up), "1.0");

            Handles.color = ColorLib.Gray(35);
            for (var i = 1; i < baseRect.width / 50; i++)
            {
                var t = i / (baseRect.width / 50) * previewDuration;
                t *= (baseRect.width / 50.0f) / (int)(baseRect.width / 50);

                Handles.DrawLine(fromGraphPos(new Vector2(t / previewDuration, min)), fromGraphPos(new Vector2(t / previewDuration, max)));
                Handles.Label(fromGraphPos(new Vector2(t / previewDuration, min)), $"{t:G3}s");
            }

            Handles.color = Color.green;
            for (var i = 0; i < data.Count - 1; i++)
            {
                var a = fromGraphPos(data[i]);
                var b = fromGraphPos(data[i + 1]);

                Handles.DrawLine(a, b);
            }
        }

        private static List<Vector2> GetGraphData(float f, float z, float r, out float min, out float max, float duration)
        {
            var data = new List<Vector2>();
            min = float.MaxValue;
            max = float.MinValue;

            var driver = new DampedSpring(f, z, r);

            var samples = (int)(duration * 50.0f);

            for (var i = 0; i < samples; i++)
            {
                var t = (i / (float)samples * duration);
                var dt = duration / samples;

                driver.Update(1.0f, null, dt);

                data.Add(new Vector2(t / duration, driver.PositionF));

                if (driver.PositionF > max) max = driver.PositionF;
                if (driver.PositionF < min) min = driver.PositionF;
            }

            return data;
        }
    }
}
