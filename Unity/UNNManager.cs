using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UNNManager : MonoBehaviour
{
    // kind of robot to use
    public int robot_model;
    private const int BAM = 0;
    private const int Braccio = 1;
    private const int DoF2 = 2;

    // ball on the gutter
    public GameObject ball;
    // desired position change frequency
    public float change_freq;
    // Desired effector position
    public GameObject target;
    // move speed for the BAM effector
    public float moveSpeed;
    // desired position on the gutter
    public GameObject target_pos;
    // should do long (0) or short test (1)
    public int longTest;


    private static float gutterLength;
    private static float leverLength;
    private static float refPositionEffector;



    private Transform effector;
    private Transform gutter;

    //positive reward range
    private static float positive_range = 0.4f;
    private Rigidbody rb;
    

    private float desired_ball_position;
    private int nb_step_change;

    public BaseRobot robot_controller;
    private TestHandler test_handler;
    private RobotJoint[] robot; 

    private int i;


    public float[] getObs(){ 
        // collect observations, normalize and return a observation array
        // change the desired position every change_freq steps
        // For a test, the desired ball posiion is contain in an array
        // else it is randomly sampled
        if(nb_step_change%change_freq==0){
            if(test_handler.test) {
                desired_ball_position = test_handler.test_video[i%3] * longTest + test_handler.vector_test_desired_pos[i] * (1 - longTest); 
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
        // initialize the manager 
        test_handler = GetComponent<TestHandler>();
        float[] init_angles;
        if (robot_model != BAM){
            if(robot_model == Braccio) init_angles = new float[]{35,59,-90};
            else init_angles = new float[]{60f,-60f};
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
        // move and rotate the gutter according to the effector height
        float h = effector.position.y - refPositionEffector;
        float alpha = (180f * Mathf.Asin(h/leverLength))/Mathf.PI;
        gutter.transform.rotation = Quaternion.Euler(0f, 0f,-alpha);

    }

    

    public void setInstruction(float instruction){
        // get command from the UNNAgent and execute it on the chosen robot
        // update gutter state afterwards
        if(robot_model!= BAM){
            float instruction_pos = instruction + refPositionEffector;
            target.transform.position = new Vector3(target.transform.position.x,instruction_pos,target.transform.position.z);
            if(robot_model == Braccio) robot_controller.InverseKinematic3D(target.transform.position);
            else robot_controller.InverseKinematic2D(target.transform.position.y);
        }
        else {
            float effector_position = Mathf.MoveTowards(effector.position.y,refPositionEffector + instruction, moveSpeed * Time.deltaTime);
            effector.position = new Vector3(effector.position.x,effector_position,effector.position.z);  
        }
        moveGutter();
    }

    public void initBallPosition(){
        // initialize ball position with a random uniform (training) or predefined (test)
        float max_height = Mathf.Max(1.5f,effector.position.y);
        float ball_abscisse;
        if(test_handler.test) ball_abscisse = 0.25f*gutterLength;
        else ball_abscisse = Random.Range(0.15f, 0.85f)*gutterLength;
        ball.transform.localPosition = new Vector3(ball_abscisse,max_height+0.1f,0f);
    }


    public float computeReward(){
        // compute the agent reward for the timestep
        // reward is the weighted distance between desired ball position and current ball position 
        // a small positive reward if the ball is in the positive reward range
        float reward = 0f;
        float distance = Vector3.Distance(ball.transform.localPosition,new Vector3(desired_ball_position*gutterLength,0f,0f));
        if(distance <= positive_range){
            reward = 0.3f;
        }
        else reward = -0.4f * distance;
        return reward;
    }


        public bool end_episode(){
        if(ball.transform.position.y < 0.3f){
            rb.velocity = new Vector3(0f,0f,0f);
            //Debug.Log("BALL DROPPED !!!");
            return true;
        }
        return false;
        }
}


