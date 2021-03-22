using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class RobotJoint{

    public RobotJoint Parent; 
    public Transform JointBody; 
    public Transform EndPoint; 
    public Vector3 MoveAxis; 
    public float JointValue; 
    public float Length;

    public RobotJoint(Transform body, float length, Vector3 move_axis){
        JointBody = body; 
        Length = length; 
        MoveAxis = move_axis; 
    } 

    public void SetEndPoint(Transform end){
        EndPoint = end; 
    }

    public void UpdateJoint(float joint_val){
        JointValue = joint_val;
        Quaternion initial_rot = Quaternion.identity;
        if(Parent != null){
            JointBody.position = Parent.GetEnd();
            initial_rot = Parent.JointBody.rotation;
        }
        JointBody.rotation = initial_rot * Quaternion.AngleAxis(JointValue, MoveAxis);
    }

    public Vector3 GetEnd(){
        // return JointBody.position + Quaternion.AngleAxis(JointValue, MoveAxis) * JointBody.up * Length; 
        return EndPoint.position; 
    }

    public void SetParent(RobotJoint parent){
        Parent = parent; 
    }

}

//[ExecuteInEditMode]
public class BaseRobot : MonoBehaviour
{

    private float[] JointValues;
    private Transform tmp;

    // Robot caracteristic
    public float ARM_RANGE;
    public float MIN_RANGE;
    public float[] lengths;
    public Vector3 ORIGIN;


    [Header("Control Params")]
    public int NbJoints;  
    public float[] JointTargets; 
    public float JointSpeed;

    [Header("Morpho")]
    public RobotJoint[] Joints;
    public Transform effector;


    [Header("Creation Params")]
    public int Creation_Joints; 
    public float[] Creation_Length; 
    public Vector3 Creation_RotationAxis; 
    public GameObject JointViz;
    public GameObject Grip;


    [ContextMenu("Create Robot")]
    void CreateRobot(){
        float sum =0;
        RobotJoint[] procedural_joints = new RobotJoint[Creation_Joints]; 
        for(int i = 0; i<Creation_Joints; i++){
            GameObject g = new GameObject(); 
            g.name = "J" + i.ToString();
            g.transform.position = Vector3.zero +  Vector3.up * sum;
            procedural_joints[i] = new RobotJoint(g.transform, Creation_Length[i], Creation_RotationAxis); 
            if(i > 0)
                procedural_joints[i].SetParent(procedural_joints[i-1]); 
            // VIZ

            GameObject g_viz = Instantiate(JointViz, g.transform.position, Quaternion.identity) as GameObject; 
            g_viz.transform.SetParent(g.transform); 
            g_viz.transform.GetChild(0).gameObject.transform.localScale = new Vector3(0.2f, Creation_Length[i] , 0.2f);
            Debug.Log("localPosition : "+g_viz.transform.GetChild(0).gameObject.transform.localPosition);
            g_viz.transform.GetChild(0).gameObject.transform.localPosition = g_viz.transform.GetChild(0).gameObject.transform.localPosition + new Vector3(0f,(Creation_Length[i] - 1)/2,0f);
            Debug.Log("inc : "+new Vector3(0f,(Creation_Length[i] - 1)/2,0f));
            GameObject end_point = new GameObject(); 
            end_point.transform.position = g.transform.position + Vector3.up * Creation_Length[i];
            end_point.transform.SetParent(g.transform); 
            procedural_joints[i].SetEndPoint(end_point.transform); 
            sum += Creation_Length[i];
            tmp = g.transform;
        }
        Joints = procedural_joints;
        GameObject grip_viz = Instantiate(Grip) as GameObject;  
        grip_viz.transform.position = Vector3.zero + Vector3.up * sum;
        grip_viz.transform.SetParent(tmp);
    }

    // Start is called before the first frame update
    void Start()
    {
       // InitializeRobot(); 

    }



    // Update is called once per frame
    void Update()
    {
        UpdateAngles();   
        UpdateRobot(); 
    }

    public RobotJoint[] InitializeRobot(float[] init_angles){
        Debug.Log("Init robot");
        NbJoints = Joints.Length; 
        JointValues = new float[NbJoints]; 
        JointTargets = new float[NbJoints];
        for(int i = 1; i< Joints.Length; i++)
            Joints[i].SetParent(Joints[i-1]); 
        ARM_RANGE = effector.transform.position.y;
        MIN_RANGE = Joints[1].GetEnd().y*1.2f;
        ORIGIN = Joints[0].JointBody.position;
        initLengths();
        for(int i = 0;i<init_angles.Length-1;i++){
            JointValues[i] = init_angles[i];
            JointTargets[i] = init_angles[i];
        } 
        UpdateAngles();
        return Joints;
    }

    void initLengths(){
        lengths = new float[Creation_Joints];
        for(int i = 0;i<Creation_Joints;i++){
            if(i==Creation_Joints-1){
                lengths[i] = Vector3.Distance(Joints[i].JointBody.position,effector.position);   
            }
            else{
                lengths[i] = Vector3.Distance(Joints[i].JointBody.position,Joints[i+1].JointBody.position);
            }
        }
    }

    public virtual void UpdateRobot(){
        for(int i = 0; i<NbJoints; i++)
            Joints[i].UpdateJoint(JointValues[i]); 
    }

    public virtual void UpdateAngles(){
        for(int i = 0; i<JointValues.Length; i++){
            JointValues[i] = Mathf.MoveTowards(JointValues[i], JointTargets[i], JointSpeed * Time.deltaTime);  
        }
        return; 
    }

    public virtual void SetAngles(float[] targets){
        JointTargets = targets;
        return; 
    }

    public virtual float[] GetAngles(){
        return JointValues; 
    }

    public void InverseKinematic3D(Vector3 targetPosition){
            float[] consigne = new float[3];
            float[] angles = new float[4];
            angles[0] = 0;
            float x3 = - targetPosition.x + ORIGIN.x;
            float y3 = targetPosition.y - lengths[3] - lengths[0];
            float r3_sqr = x3*x3 + y3*y3;

            float psi3 = Mathf.Acos((lengths[1]*lengths[1] + lengths[2]*lengths[2] - r3_sqr)/(2*lengths[1]*lengths[2]));
            consigne[1] = Mathf.PI - psi3;

            float psi2 = Mathf.Atan(x3/y3);
            consigne[0] = psi2 - Mathf.Atan((lengths[2]*Mathf.Sin(consigne[1]))/(lengths[1]+lengths[2]*Mathf.Cos(consigne[1])));
            consigne[2] = - consigne[0] - consigne[1];

            for(int i=1;i<4;i++){
                if(i==1) angles[i] = Mathf.Round(Mathf.Clamp(consigne[i-1] * 180f/Mathf.PI,-90,90));
                else angles[i] = Mathf.Round(Mathf.Clamp(consigne[i-1] * 180f/Mathf.PI,-90,90));
            }
            SetAngles(angles);
    }

    public void InverseKinematic2D(float targetPosition){
        float[] consigne = new float[3];
        consigne[0] = 0;
        var tmp = (targetPosition - lengths[0] - lengths[2])/lengths[1];
        consigne[1] = Mathf.Acos(tmp);
        consigne[1] = Mathf.Clamp(consigne[1]*(180/Mathf.PI),-90,90);
        consigne[2] = - consigne[1];
        SetAngles(consigne);
    }

    void OnDrawGizmos(){
        if(Joints != null){
            if(Joints.Length > 1){
                Gizmos.color = Color.red; 
                for(int i = 0; i<Joints.Length;i ++)
                    Gizmos.DrawLine(Joints[i].JointBody.position, Joints[i].GetEnd()); 
            }
        }
    }
}
