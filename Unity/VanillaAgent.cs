using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using MLAgents.Sensors;
using System.IO;
using System;

public class VanillaAgent : Agent
{
    //TODO : rajouter le paramètre de range de l'effecteur + recevoir refPos de initialize 
    public  int robot_model;
    private float delay;
    private float delay_counter;
    public bool log;
    public bool delayAware;
    
    private VanillaManager manager;
    private TestHandler test_handler;   

    
    private Queue<float[]> bufferObservations;
    private static int delayChangeFreq = 15;
    private  int nbStepDelayChange;
    private static float STEP;
    private float episode_reward;
    private float rew;

    private float[] REF_POSITION_JOINTS;
    private MLAgents.Policies.BehaviorParameters bP;


    public override void Initialize()
    {
        float ACADEMY_STEP = 0.02f;
        int DECISION_REQUEST = 5;
        STEP = ACADEMY_STEP*DECISION_REQUEST;
        bP = GetComponent<MLAgents.Policies.BehaviorParameters>(); 
        var brainP = bP.brainParameters;
        manager = GetComponent<VanillaManager>();
        test_handler = GetComponent<TestHandler>();
        if(log){
            test_handler.init();
        }
        if(robot_model == 1) REF_POSITION_JOINTS = new float[]{35,59,-90};
        else REF_POSITION_JOINTS = new float[]{60f,-60f};
        string[] arguments = Environment.GetCommandLineArgs();
        for(int i=0;i<arguments.Length;i++){
            if(Equals(arguments[i],"--delay")){
                delay = float.Parse(arguments[i+1]);
                delay_counter = delay/STEP;
                test_handler.test = true;
                test_handler.train = false;
                manager.change_freq = 100;
            }
            if(Equals(arguments[i],"--aware")) delayAware = int.Parse(arguments[i+1])==0;
        }
        if(delayAware) brainP.vectorObservationSize = 5;
        else {
            brainP.vectorObservationSize = 4;
        } 
        manager.init(robot_model,REF_POSITION_JOINTS);
        if(log){
            test_handler.init();
        }
        bufferObservations = new Queue<float[]>();
    }

    public override void OnEpisodeBegin()
    {   
        manager.initBallPosition();
        if(!test_handler.test){
            if(nbStepDelayChange%delayChangeFreq==0){
                delay = UnityEngine.Random.Range(0.1f, 1f);;
                delay_counter = delay/STEP;
                bufferObservations.Clear();
            }   
            nbStepDelayChange++;
            test_handler.incNbEpisode(episode_reward);
        }
        episode_reward = 0;
    }
  
    public override void CollectObservations(VectorSensor sensor)
    {
        float[] obs = manager.getObs();
        sensor.AddObservation(obs[3]); 
        if(delayAware) sensor.AddObservation(delay);
        if(delay > 0) bufferObservations.Enqueue(obs);
        if(bufferObservations.Count >= delay_counter && (test_handler.test && !delayAware)){
            var delayedObs = bufferObservations.Peek();
            bufferObservations.Dequeue();
            sensor.AddObservation(delayedObs[0]);
            sensor.AddObservation(delayedObs[1]);
            sensor.AddObservation(delayedObs[2]);
            test_handler.logObs(obs,delayedObs);
        }
        else if(bufferObservations.Count >= delay_counter && delayAware){
            var delayedObs = bufferObservations.Peek();
            bufferObservations.Dequeue();
            sensor.AddObservation(delayedObs[0]);
            sensor.AddObservation(delayedObs[1]);
            sensor.AddObservation(delayedObs[2]);
            test_handler.logObs(obs,delayedObs);
        }
        else if(!delayAware && test_handler.train) {
            sensor.AddObservation(obs[0]);
            sensor.AddObservation(obs[1]);
            sensor.AddObservation(obs[2]);
        }
    }



    public override void OnActionReceived(float[] vectorAction)
    {
        float[] consigne = new float[4];
        consigne[0] = 0;
        if(robot_model == 1){
            consigne[1] = Mathf.Clamp(vectorAction[0], -1f, 1f)*10f+REF_POSITION_JOINTS[0];
            consigne[2] = Mathf.Clamp(vectorAction[1], -1f, 1f)*30f+REF_POSITION_JOINTS[1];
            consigne[3] = Mathf.Clamp(vectorAction[2], -1f, 1f)*30f+REF_POSITION_JOINTS[2];
        }
        else{
            consigne[1] = Mathf.Clamp(vectorAction[0], -1f, 0.5f)*60f+REF_POSITION_JOINTS[0];
            consigne[2] = Mathf.Clamp(vectorAction[1], -0.5f, 1f)*60f+REF_POSITION_JOINTS[1];
        }
        
        manager.setInstruction(consigne);
        if(manager.end_episode()){
            EndEpisode();
        }
        else{
            rew = manager.computeReward();
            episode_reward+=rew;
        }
        AddReward(rew); 
    }

    public override float[] Heuristic()
    {
        var action = new float[3];
        action[0] = Input.GetAxis("Vertical");      
        action[1] = Input.GetAxis("Horizontal");    
        return action;
    }

    void OnApplicationQuit()
    {
        if(log) test_handler.closeFile();
    }
}

