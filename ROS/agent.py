import tensorflow as tf
import numpy as np
from std_msgs.msg import Float32MultiArray
import math

# This class load the TF model and predict the actions

class Agent():
  def __init__(self,model_path):
    self.reward = 0
    self.graph = self.load_graph(model_path)  
    self.model_input = self.graph.get_tensor_by_name('prefix/vector_observation:0')
    self.model_output = self.graph.get_tensor_by_name('prefix/action:0')
    self.tf_session = tf.Session(graph=self.graph)

  def predict(self,obs):
    y_out = self.tf_session.run(self.model_output, feed_dict={
          self.model_input: [obs]
      })
    return y_out

  def act(self,obs):
    out = self.predict(obs)      
    return out[0]

  def end_episode(self):
    self.reward = 0


  def load_graph(self,frozen_graph_filename):
      with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
          graph_def = tf.GraphDef()
          graph_def.ParseFromString(f.read())

      with tf.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name="prefix")
      return graph

