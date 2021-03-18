using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class TestHandler : MonoBehaviour
{
    public string test_name;
    public int test_length;
    public bool test;
    public bool train;
	private int nb_episode = -1;
	private StreamWriter sw;
	private static string root_path = "/Users/admin/Documents/ml_dev/Pytorch/Unity/gutter_task/Assets/tests_article/";

    // desired poistion for long test.
    [HideInInspector] public float[] vector_test_desired_pos = new float[]{0.665146f, 0.319444f, 0.210018f, 0.788669f, 0.625432f, 0.413984f, 
        0.492742f, 0.217814f, 0.592103f, 0.709749f, 0.573616f, 0.787164f, 0.263239f, 0.270857f, 0.546654f, 0.211952f, 0.586792f, 0.629471f, 
        0.333639f, 0.20229f, 0.614752f, 0.785692f, 0.240727f, 0.304158f, 0.515807f, 0.379363f, 0.716785f, 0.487727f, 0.360066f, 0.506695f, 0.680391f, 
        0.671943f, 0.551617f, 0.494168f, 0.562634f, 0.271272f, 0.648575f, 0.322888f, 0.368295f, 0.645345f, 0.274886f, 0.637379f, 0.480301f, 0.286186f,
         0.483854f, 0.252525f, 0.653358f, 0.720112f, 0.433729f, 0.775511f};

    // desired position for short test (videos)
    [HideInInspector] public float[] test_video = new float[]{0.3f,0.8f,0.5f}; 

  
    public void init(){
        if(train) test_length = 0;
        try
        {
            sw = new StreamWriter(root_path+test_name);
        }
        catch(IOException e)
        {
            Debug.Log("Exception while opening file : " + e.Message);
        }
        finally
        {
            //Debug.Log(sw);
        }
    }

    public void logObs(float[] obs,float[] delayed_obs){
		try
        {
            if(nb_episode < test_length){
                sw.WriteLine("ball_pos : "+obs[0]);
                sw.WriteLine("ball_vel : "+obs[1]);
                sw.WriteLine("effec_pos : "+obs[2]);
                sw.WriteLine("des_ball_pos : "+obs[3]);
                sw.WriteLine("delayed_ball_pos : "+delayed_obs[0]);
                sw.WriteLine("delayed_ball_vel : "+delayed_obs[1]);
                sw.WriteLine("delayed_effec_pos : "+delayed_obs[2]);
                sw.WriteLine("delayed_des_ball_pos : "+delayed_obs[3]);
            }
        }
        catch(IOException e)
        {
            Debug.Log(e.Message);
        }
        finally
        {
            //Debug.Log(sw);
        } 
    }

    public void logActions(float action){
        try
        {
            if(nb_episode < test_length){
                sw.WriteLine("unn_instruction : "+action);
                
            }
        }
        catch(IOException e)
        {
            Debug.Log(e.Message);
        }
        finally
        {
            //Debug.Log(sw);
        } 
    }

    public void incNbEpisode(float reward){
    	try
        {
        	nb_episode+=1;
            if(train){
                sw.WriteLine("nb_episode : "+nb_episode);
                sw.WriteLine("rew_ep : "+reward);
            }
        }
        catch(IOException e)
        {
            Debug.Log(e.Message);
        }
        finally
        {
            //Debug.Log(sw);
        } 
    }

    public void closeFile(){
    	Debug.Log("closing file");
        try
        {
            if(sw != null){
                sw.Close();
            }
        }
        catch(IOException e)
        {
            Debug.Log("Exception: " + e.Message);
        }
        finally
        {
            //Debug.Log("Executing finally block.");
        }
    }
}
