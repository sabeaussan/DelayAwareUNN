using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using MLAgents.Sensors;
using System.IO;

public class UNNAgent : Agent
{
    
    // Delay observed by the model
    private float delay;

    // Delay in timestep
    private  float delay_counter;

    public bool log;

    // should train/test delay aware
    public bool delayAware;
    
    private Manager manager;
    private TestHandler test_handler;
    
    private Queue<float[]> bufferObservations;

    // frequency at which the delay is changed
    private static int delayChangeFreq = 15;
    private  int nbStepDelayChange;


    private static float STEP;
    private float episode_reward;
    private float rew;


    public override void Initialize()
    {
        // Initialize test handler, manager and buffer
        manager = GetComponent<Manager>();
        manager.init();
        test_handler = GetComponent<TestHandler>();
        if(log){
            test_handler.init();
        }
        bufferObservations = new Queue<float[]>();
        float ACADEMY_STEP = 0.02f;
        int DECISION_REQUEST = 5;
        STEP = ACADEMY_STEP*DECISION_REQUEST;

    }

    public override void OnEpisodeBegin()
    {   
        // Initialize the position of the ball at the begining of the episode
        manager.initBallPosition();
        if(!test_handler.test){
            if(nbStepDelayChange%delayChangeFreq==0){
                // Sample a new delay from a Uniform distribution
                delay = Random.Range(0.1f, 1f);;
                delay_counter = delay/STEP;
                // clear buffer with delayed obs at the begining of each episode
                bufferObservations.Clear();
            }   
            nbStepDelayChange++;
            test_handler.incNbEpisode(episode_reward);
        }
        else{
            // fixed delay for tests
            delay = 0.7f; 
            delay_counter = delay/STEP; 
        }
        episode_reward = 0;
    }
 
    public override void CollectObservations(VectorSensor sensor)
    {
        // collect observations
        float[] obs = manager.getObs();
        sensor.AddObservation(obs[3]); 
        if(delayAware) sensor.AddObservation(delay);
        // add obs to the buffer
        if(delay > 0) bufferObservations.Enqueue(obs);
        // if buffer is full, get delayed obs
        if(bufferObservations.Count >= delay_counter && delayAware){
            var delayedObs = bufferObservations.Peek();
            bufferObservations.Dequeue();
            sensor.AddObservation(delayedObs[0]);
            sensor.AddObservation(delayedObs[1]);
            sensor.AddObservation(delayedObs[2]);
            if(test_handler.test) test_handler.logObs(obs,delayedObs);
        }
        else if(!delayAware) {
            sensor.AddObservation(obs[0]);
            sensor.AddObservation(obs[1]);
            sensor.AddObservation(obs[2]);
            if(test_handler.test) test_handler.logObs(obs,obs);
        }
    }



    public override void OnActionReceived(float[] vectorAction)
    {
        // get the action return by the neural network, clip it and scale it
        float consigne =  Mathf.Clamp(vectorAction[0], -1f, 1f) *(manager.robot_model == 2 ? 0.5f:0.6f);
        manager.setInstruction(consigne);

        if(manager.end_episode()){
            EndEpisode();
        }

        // compute reward for the step
        rew = manager.computeReward();
        episode_reward+=rew;
        AddReward(rew);
    }

    public override float[] Heuristic()
    {
        var action = new float[1];
        action[0] = Input.GetAxis("Vertical");      
        return action;
    }

    void OnApplicationQuit()
    {
        if(log) test_handler.closeFile();
    }
}
