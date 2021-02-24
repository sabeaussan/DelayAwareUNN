using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VanillaManager : MonoBehaviour
{

    private int robot_model;
    public GameObject ball;
    public float change_freq;
    public GameObject target_pos;
    private const int Braccio = 1;
    private const int DoF2 = 2;


    private static float gutterLength;
    private static float leverLength;
    private static float refPositionEffector;
    private Transform effector;
    private Transform gutter;

    private static float positive_range = 0.4f;
    private Rigidbody rb;
    

    private float desired_ball_position;
    private int nb_step_change;

    public BaseRobot robot_controller;
    private RobotJoint[] robot; 
    private TestHandler test_handler;

    private int i;


    public void init(int p_robot_model,float[] init_angles){
        test_handler = GetComponent<TestHandler>();
        robot_model = p_robot_model;
        rb = ball.GetComponent<Rigidbody>();
        robot = robot_controller.InitializeRobot(init_angles);  
        effector = robot_controller.effector;
        gutter = this.gameObject.transform.GetChild(0);
        refPositionEffector = gutter.position.y;
        leverLength = 5.4f;
        gutterLength = 5.6f;
        Debug.Log(refPositionEffector);
    } 

    public void setInstruction(float[] actions){
        robot_controller.SetAngles(actions);
        moveGutter();
    }

    public float[] getObs(){ 
        if(nb_step_change%change_freq==0){
            if(test_handler.test) {
                desired_ball_position = test_handler.vector_test_desired_pos[i];
                i++;
            }
            else  desired_ball_position = Random.Range(0.15f, 0.85f); 
        }
        nb_step_change++;
        target_pos.transform.localPosition = new Vector3(desired_ball_position*gutterLength,0.4f,0f);
        float[] obs = new float[4]; 
        obs[0] = ball.transform.localPosition.x/gutterLength;
        obs[1] = rb.velocity.x/6f;
        obs[2] = (robot_controller.effector.position.y-refPositionEffector)/2; // pas sur que ce soit bien normalizé
        obs[3] = desired_ball_position;
        return obs;
    }

    public void moveGutter(){
        float h = effector.position.y - refPositionEffector;
        float alpha = (180f * Mathf.Asin(h/leverLength))/Mathf.PI;
        gutter.transform.rotation = Quaternion.Euler(0f, 0f,-alpha);

    }

    public void initBallPosition(){
        float max_height = Mathf.Max(1.5f,effector.position.y);
        float ball_abscisse;
        if(test_handler.test) ball_abscisse = 0.25f*gutterLength;
        else ball_abscisse = Random.Range(0.15f, 0.85f)*gutterLength;
        ball.transform.localPosition = new Vector3(ball_abscisse,max_height+0.1f,0f);
    } 

    public float get_signed_angle(float rotation){
        if(rotation > 265){
            rotation = rotation - 360f;
        }
        else if(rotation < - 265) {
            rotation = rotation + 360f;
        }
        return rotation;
    }

    public float computeReward(){
        float reward = 0f;
        float distance = Vector3.Distance(ball.transform.localPosition, new Vector3(desired_ball_position*gutterLength,0f,0f));
        if(distance <= positive_range){
            reward = 0.3f;
        }
        else reward = -0.4f * distance - 0.03f * Mathf.Abs(get_signed_angle(robot_controller.effector.rotation.eulerAngles.x));
        return reward;
    }


    public bool end_episode(){
        if(ball.transform.position.y < 0f){
            rb.velocity = new Vector3(0f,0f,0f);
            //Debug.Log("BALL DROPPED !!!");
            return true;
        }
        return false;
    }
}


