using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor; //is wheel attached to motor
    public bool steering; //does this wheel apply steering angle
}

public class CarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;


    void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0) return;

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.position = position;
        visualWheel.rotation = rotation;
    }


    private void FixedUpdate()
    {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");

        foreach (var axle in axleInfos)
        {
            if (axle.steering)
            {
                axle.leftWheel.steerAngle = steering;
                axle.rightWheel.steerAngle = steering;
            }

            if (axle.motor)
            {
                axle.leftWheel.motorTorque = motor;
                axle.rightWheel.motorTorque = motor;
            }

            ApplyLocalPositionToVisuals(axle.leftWheel);
            ApplyLocalPositionToVisuals(axle.rightWheel);
        }
    }
}
