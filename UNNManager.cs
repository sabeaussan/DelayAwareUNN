using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UNNManager : MonoBehaviour
{
    
    public int robot_model;
    public GameObject ball;
    public float change_freq;
    public GameObject target;
    public float moveSpeed;
    
    public GameObject target_pos;

    private static float gutterLength;
    private static float leverLength;
    private static float refPositionEffector;

    private const int BAM = 0;
    private const int Braccio = 1;
    private const int DoF2 = 2;

    private Transform effector;
    private Transform gutter;

    private static float positive_range = 0.4f;
    private Rigidbody rb;
    

    private float desired_ball_position;
    private int nb_step_change;

    public BaseRobot robot_controller;
    private TestHandler test_handler;
    private RobotJoint[] robot; 

    private int i;


    public float[] getObs(){ 
        if(nb_step_change%change_freq==0){
            if(test_handler.test) {
                desired_ball_position = test_handler.test_video[i];
                i++;
            }
            else  desired_ball_position = Random.Range(0.15f, 0.85f);
        }
        target_pos.transform.localPosition = new Vector3(desired_ball_position*gutterLength,0.4f,0f);
        nb_step_change++;
        float[] obs = new float[4];
        obs[0] = ball.transform.localPosition.x/gutterLength;
        obs[1] = rb.velocity.x/6f;
        obs[2] = (effector.position.y-refPositionEffector)/0.6f; // à changer pour plus général
        obs[3] = desired_ball_position; 
        return obs;
    }

    public void init(){ 
        test_handler = GetComponent<TestHandler>();
        float[] init_angles;
        if (robot_model != BAM){
            if(robot_model == Braccio) init_angles = new float[]{0f,-35,-59,90};
            else init_angles = new float[]{0f,60f,-60f};
            robot = robot_controller.InitializeRobot(init_angles); 
        } 
        if(robot_model == BAM) effector = this.gameObject.transform.GetChild(1); 
        else  effector = robot_controller.effector;
        gutter = this.gameObject.transform.GetChild(0);
        refPositionEffector = gutter.position.y;
        leverLength = 5.4f;
        rb = ball.GetComponent<Rigidbody>();
        gutterLength = 5.6f;
    } 

    public void moveGutter(){
        float h = effector.position.y - refPositionEffector;
        float alpha = (180f * Mathf.Asin(h/leverLength))/Mathf.PI;
        gutter.transform.rotation = Quaternion.Euler(0f, 0f,-alpha);

    }

    // penser a mod range 2DoF

    public void setInstruction(float instruction){
        if(robot_model!= BAM){
            float instruction_pos = instruction + refPositionEffector;
            target.transform.position = new Vector3(target.transform.position.x,instruction_pos,target.transform.position.z);
            if(robot_model == Braccio) robot_controller.GeometricMethod(target.transform.position);
            else robot_controller.InverseKinematic2D(target.transform.position.y);
        }
        else {
            float effector_position = Mathf.MoveTowards(effector.position.y,refPositionEffector + instruction, moveSpeed * Time.deltaTime);
            effector.position = new Vector3(effector.position.x,effector_position,effector.position.z);  
        }
        moveGutter();
    }

    public void initBallPosition(){
        float max_height = Mathf.Max(1.5f,effector.position.y);
        float ball_abscisse;
        if(test_handler.test) ball_abscisse = 0.25f*gutterLength;
        else ball_abscisse = Random.Range(0.15f, 0.85f)*gutterLength;
        ball.transform.localPosition = new Vector3(ball_abscisse,max_height+0.1f,0f);
    }


    public float computeReward(){
        float reward = 0f;
        float distance = Vector3.Distance(ball.transform.localPosition,new Vector3(desired_ball_position*gutterLength,0f,0f));
        if(distance <= positive_range){
            reward = 0.3f;
        }
        else reward = -0.4f * distance;
        return reward;
    }


    public bool end_episode(){
        return false;//ball.transform.position.y < Mathf.Min(effector1.transform.position.y,effector2.transform.position.y) - 0.1f;
    }
}


