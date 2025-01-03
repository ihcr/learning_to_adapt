import numpy as np
import torch
import torch.nn as nn
from torch.distributions.categorical import Categorical

import learning_to_adapt.network_module as module


class PolicyManager:
    def __init__(self, weights_path, mean_path, var_path, ob_dim, act_dim, action_mean, action_std, policy_net, activation=nn.LeakyReLU, action_type='cont'):
        # set up network parameters
        self.ob_dim = ob_dim
        self.act_dim = act_dim
        self.action_mean = np.array(action_mean)
        self.action_std = np.ones(self.act_dim)*action_std
        self.policy_net = policy_net
        self.activation = activation
        
        # set up network
        self.loaded_graph = module.MLP(self.policy_net, activation, self.ob_dim, self.act_dim)
        self.loaded_graph.load_state_dict(torch.load(weights_path, map_location=torch.device('cpu'))['actor_architecture_state_dict'])
        
        # load scaling
        self.count = 1e5
        self.mean = np.loadtxt(mean_path, dtype=np.float32)
        self.var = np.loadtxt(var_path, dtype=np.float32)
        self.epsilon = (np.ones(self.ob_dim) * 1e-8).astype(np.float32)
        
        # set action type
        self.action_type = action_type
        
    def get_action(self, input_obs, normalise_obs=True, scale_action=True):
        # normalise obs if required
        if normalise_obs:
            obs = self.normalise_obs(input_obs)
        else:
            obs = input_obs
        
        # get unscaled action
        action_ll = self.loaded_graph.architecture(torch.from_numpy(obs).cpu())
        # scale action if required
        if scale_action and self.action_type == 'cont':
            action_scaled = action_ll.cpu().detach().numpy().astype(np.float64)
            action_scaled = action_scaled * self.action_std
            action_scaled = action_scaled + self.action_mean
            return action_scaled
        elif self.action_type == 'disc':
            probs = Categorical(logits=action_ll)
            action_ll = probs.sample()
            action_ll = action_ll.cpu().detach().numpy().astype(np.float32)
            return action_ll
        else:
            return action_ll
            
    def normalise_obs(self, obs):
        obs_norm = (obs.astype(np.float32) - self.mean)/np.sqrt(self.var + self.epsilon)
        return obs_norm
        
    def update_action_mean(self, new_action_mean):
        self.action_mean = new_action_mean.copy()
