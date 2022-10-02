import numpy as np
import matplotlib.pyplot as plt

import torch
from torch import nn
from torch import optim
import torch.distributions as D
import torch.nn.functional as F

import itertools
from copy import deepcopy

from torch.utils.tensorboard import SummaryWriter
import sys
import pickle


# HYPERPARAMETERS
lr_pi = 1e-3
lr_q = 3e-3
gamma = 0.99
alpha = 0.1
polyak = 0.995
batch_size = 65536
steps_per_rollout = 25

# N.B. steps/rollout must divide graph_interval evenly
graph_interval = 2500
assert (graph_interval//steps_per_rollout)*steps_per_rollout == graph_interval

nrollouts = 500
rolloutlen = 20
randrollouts1 = nrollouts//10
randrollouts2 = nrollouts//100

q_hid = 128
pi_hid = 16


LOG_STD_MIN, LOG_STD_MAX = -20, 2

RUN_NAME = sys.argv[1]

track = np.load("trackdata.npy")
M = track.shape[0]
TRACK_HALFWIDTH = .7

M = track.shape[0]
Ttrack = torch.tensor(track, dtype=torch.float32).to("cuda")
dtrack = track[:, :2] - np.roll(track[:, :2], -1, axis=0)
ds = np.linalg.norm(dtrack, axis=1)
# used for reward calc
dssum = torch.tensor(np.cumsum(np.concatenate([[0], ds, ds])),
                     dtype=torch.float32, device="cuda")


class CarEnv:
    def __init__(self, mk1=28.8, mk2=1.35, maxa=18):
        self.mk1 = mk1
        self.mk2 = mk2
        self.mk3 = 0.42*5
        self.mk4 = 0.17*3
        self.maxa = maxa
        self.brake_k2, self.brake_k3 = 1.23, 0.875

    def randstate(self, N):
        si = np.random.randint(0, M, (N,))

        ss = np.random.randn(9*N).reshape(-1, 9)

        ss[:, 2] = 0.2*ss[:, 2] + np.arctan2(track[si, 3], track[si, 2])  # theta
        ss[:, 3] = np.abs(ss[:, 3])*4  # velocity
        ss[:, 4] = 0
        ss[:, 5] = np.abs(ss[:, 0])*0.2
        ss[:, 6] *= TRACK_HALFWIDTH/2.5

        R = np.stack([track[si, 2:4], np.array([-track[si, 3], track[si, 2]]).T], axis=1)
        ss[:, :2] = np.einsum('jik,ji->jk', R, ss[:, 5:7]) + track[si, :2]
        CS = np.stack([np.cos(ss[:, 2]), np.sin(ss[:, 2])], axis=1)
        ss[:, 7:9] = np.einsum('jki,ji->jk', R, CS)

        return torch.tensor(ss, dtype=torch.float32).to("cuda"), torch.tensor(si, dtype=torch.long).to("cuda")

    # take a step in the env; returns r (reward), sp (s prime aka next state), offtrack (done-ish)
    def step(self, s, i, a, dt=1.0/30):
        # 1. step in xg, yg, v, w space
        #   0   1    2    3  4   5   6   7   8
        # (xg, yg, theta, v, w, xl, yl, cl, sl)

        sp = s.clone()
        theta = s[:, 2]
        v = s[:, 3]
        w = s[:, 4]

        u_v = a[:, 0]
        u_delta = a[:, 1]

        # handle accel
        dv = self.mk1 * u_v - self.mk2 * v * u_v - self.mk4*v
        dv[(u_v > 0) * (v > 0)] -= self.mk3

        # handle braking
        dv[u_v < 0] = self.brake_k2 * v[u_v < 0] * u_v[u_v < 0]
        dv[(u_v < 0) * (v > 0)] -= self.brake_k3

        k = u_delta*1.5
        # verlet integration of acceleration; first half
        v = v + dv*dt*0.5
        acc = v**2 * torch.abs(k)

        # max out lateral acceleration by understeering samples which are too fast for their curvature
        toofast = acc > self.maxa
        k[toofast] = torch.sign(k[toofast]) * self.maxa / v[toofast]**2

        dv -= 100*dt*torch.cos(k)
        # second half
        v = v + dv*dt*0.5
        w = 0.475*w + 0.525*(k*v)
        sp[:, 4] = w

        v[v<0] = 0
        sp[:, 3] = v
        theta = (theta+w*dt) % (2*torch.pi)
        sp[:, 2] = theta
        sp[:, 0] += v*torch.cos(theta)*dt
        sp[:, 1] += v*torch.sin(theta)*dt

        # 2. update i
        ip = i.clone()
        for _ in range(4):
            i1 = (ip+1) % M
            dxy = sp[:, :2] - Ttrack[i1, :2]
            passedi = torch.einsum('ki,ki->k', dxy, Ttrack[i1, 2:4]) >= 0
            ip[passedi] = i1[passedi]
            if not torch.any(passedi):
                break

        # 3. recompute local xl, yl, cl, sl
        dxy = sp[:, :2] - Ttrack[ip, :2]
        R = torch.stack([Ttrack[ip, 2:4], torch.stack([-Ttrack[ip, 3], Ttrack[ip, 2]]).T], axis=1)
        sp[:, 5:7] = torch.einsum('jki,ji->jk', R, dxy)
        CS = torch.stack([torch.cos(sp[:, 2]), torch.sin(sp[:, 2])], axis=1)
        sp[:, 7:9] = torch.einsum('jki,ji->jk', R, CS)
        return sp, ip


def defaultpolicy(s, i):
    k1 = Ttrack[(i + 4) % M, 4]
    #k2 = Ttrack[(i + 10) % M, 4]
    # a = v^2 k
    # v = sqrt(a/k)
    y = s[:, 6]
    dy = s[:, 8]
    targetk = -0.5*y - 3*dy + 2*k1
    targetv = torch.clip(torch.sqrt(5./(torch.abs(targetk)+.01)), 3, 10)
    #targetv = 4
    a = torch.stack([(targetv - s[:, 3])*0.2, targetk/1.5], dim=1)
    a = torch.clip(a, -1, 1)
    a += torch.randn(a.shape, device="cuda")/5
    #if np.abs(dy < TRACK_HALFWIDTH):
    #    a += np.random.randn(2)/2
    return a


def rollout(env, s, i, policy, N):
    # roll out N frames
    s0 = torch.zeros((N, *s.shape), device="cuda")
    i0 = torch.zeros((N, *i.shape), dtype=torch.long, device="cuda")
    sp = torch.zeros((N, *s.shape), device="cuda")
    ip = torch.zeros((N, *i.shape), dtype=torch.long, device="cuda")
    a = torch.zeros((N, s.shape[0], 2), device="cuda")
    r = torch.zeros((N, s.shape[0]), device="cuda")
    done = torch.zeros((N, s.shape[0]), dtype=torch.bool, device="cuda")
    for j in range(N):
        s0[j] = s
        i0[j] = i
        a[j] = policy(s, i)
        s_, i_ = env.step(s, i, a[j])
        sp[j], ip[j] = s_, i_
        trackk = Ttrack[i_, 4]
        i_[i_ < i] += M
        # s[., 5] = x along track direction
        r[j] = dssum[i_] + s_[:, 5] - dssum[i] - s[:, 5]
        # s[., 6] = y across track

        # going off track to the outside -> penalty = distance off track
        penalty = torch.abs(s_[:, 6]) > TRACK_HALFWIDTH
        # going a full track width farther -> done
        r[j, penalty] = TRACK_HALFWIDTH - torch.abs(s_[penalty, 6])
        done[j] = torch.abs(s_[:, 6]) > 2*TRACK_HALFWIDTH

        # cutting on the inside -> you're done
        # if k is negative (right turn), inside is -y
        # if k is positive, inside is +y
        done[j] |= s_[:, 6] * torch.sign(trackk) > TRACK_HALFWIDTH

        sp[j][done[j]] = s0[j][done[j]]
        s, i = sp[j], ip[j]
    # TODO: reward based on sumds[ip] - sumds[i]
    return s0, i0, a, r, sp, ip, done



class QNet(nn.Module):
    def __init__(self, M, hidden_dim=32, state_dim=6, action_dim=2):
        super().__init__()
        self.emb = nn.Embedding(M, hidden_dim)
        self.sL1 = nn.Linear(state_dim, hidden_dim)
        self.aL1 = nn.Linear(action_dim, hidden_dim)
        self.act = nn.Mish()
        self.net = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.Mish(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, s, ix, a):
        i = self.emb(ix) + self.sL1(s) + self.aL1(a)
        o = self.act(i)
        return self.net(o).squeeze(-1)


class PiNet(nn.Module):
    def __init__(self, M, hidden_dim=16, state_dim=6, action_dim=2):
        super().__init__()
        self.emb = nn.Embedding(M, hidden_dim)
        self.sL1 = nn.Linear(state_dim, hidden_dim)
        self.act = nn.Mish()
        self.net = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.Mish(),
        )
        self.mulayer = nn.Linear(hidden_dim, action_dim)
        self.logstdlayer = nn.Linear(hidden_dim, action_dim)

    def forward(self, s, ix, deterministic=False, with_logprob=True):
        o = self.act(self.emb(ix) + self.sL1(s))
        net_out = self.net(o)
        mu = self.mulayer(net_out)
        logstd = torch.clamp(self.logstdlayer(net_out), LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(logstd)

        pi_dist = D.Normal(mu, std)
        if deterministic:
            pi_action = mu
        else:
            pi_action = pi_dist.rsample()  # note rsample, not sample!

        if with_logprob:
            # tanh squashing magic copied from openai spinningup core
            logp_pi = pi_dist.log_prob(pi_action)
            logp_pi = logp_pi.sum(axis=-1)
            logp_pi -= (2*(np.log(2) - pi_action - F.softplus(-2*pi_action))).sum(axis=1)
        else:
            logp_pi = None

        pi_action = torch.tanh(pi_action)

        return pi_action, logp_pi


class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()

        self.pi = PiNet(M, pi_hid)
        self.q1 = QNet(M, q_hid)
        self.q2 = QNet(M, q_hid)

    def act(self, o, i, deterministic=False):
        with torch.no_grad():
            a, _ = self.pi(o, i, deterministic, with_logprob=False)
            return a


def qloss(o, i, a, r, d, o2, i2):
    q1 = ac.q1(o, i, a)
    q2 = ac.q2(o, i, a)

    with torch.no_grad():
        a2, logp_a2 = ac.pi(o2, i2)

        q1_pi_targ = ac_targ.q1(o2, i2, a2)
        q2_pi_targ = ac_targ.q2(o2, i2, a2)
        q_pi_targ = torch.min(q1_pi_targ, q2_pi_targ)
        backup = r + gamma * (~d) * (q_pi_targ - alpha * logp_a2)

    loss_q1 = ((q1 - backup)**2).mean()
    loss_q2 = ((q2 - backup)**2).mean()
    loss_q = loss_q1 + loss_q2

    return loss_q


def ploss(o, i):
    pi, logp_pi = ac.pi(o, i)
    q1_pi = ac.q1(o, i, pi)
    q2_pi = ac.q2(o, i, pi)
    q_pi = torch.min(q1_pi, q2_pi)

    loss_pi = (alpha * logp_pi - q_pi).mean()

    return loss_pi


# init fresh actor-critic
ac = ActorCritic().to("cuda")
ac_targ = deepcopy(ac).to("cuda")
for p in ac_targ.parameters():
    p.requires_grad = False

q_params = itertools.chain(ac.q1.parameters(), ac.q2.parameters())
q_opt = optim.Adam(q_params, lr=lr_q)
pi_opt = optim.Adam(ac.pi.parameters(), lr=lr_pi)

writer = SummaryWriter('runs/' + RUN_NAME)
step = 0
# init replay buffer w/ default policy rollouts
env = CarEnv()
s, i = env.randstate(10000)
s0, i0, a, r, sp, ip, done = rollout(env, s, i, defaultpolicy, 100)
replay_s0 = s0.view(-1, s0.shape[-1])
replay_i0 = i0.view(-1)
replay_obs1 = sp.view(-1, sp.shape[-1])[:, 3:]
replay_i1 = ip.view(-1)
replay_a = a.view(-1, a.shape[-1])
replay_r = r.view(-1)
replay_done = done.view(-1)


ravg = None
frozen = 0
replay_idx = frozen
replay_size = replay_s0.shape[0]

testlap_state = torch.zeros((1, 9), device="cuda")
testlap_state[:, :2] = Ttrack[5, :2]
testlap_idx = torch.tensor([21], dtype=torch.long, device="cuda")


def checkpoint():
    # TODO: save replay buffer also
    fname = "%s-%d.pkl" % (RUN_NAME, step)
    obj = {}
    for k, v in ac.state_dict().items():
        obj[k] = v.detach().cpu().numpy()
    pickle.dump(obj, open(fname, "wb"))
    print("checkpoint written to", fname)


while step < 10000000:
    try:
        b = torch.randint(0, replay_size, (batch_size,), device="cuda")
        q_opt.zero_grad()
        loss_q = qloss(replay_s0[b, 3:], replay_i0[b], replay_a[b], replay_r[b], replay_done[b], replay_obs1[b], replay_i1[b])
        loss_q.backward()
        q_opt.step()

        # optimize p but without gradients on q
        for p in q_params:
            p.requires_grad = False
        pi_opt.zero_grad()
        loss_pi = ploss(replay_s0[b, 3:], replay_i0[b])
        loss_pi.backward()
        pi_opt.step()
        for p in q_params:
            p.requires_grad = True

        # Finally, update target networks by polyak averaging.
        with torch.no_grad():
            for p, p_targ in zip(ac.parameters(), ac_targ.parameters()):
                # NB: We use an in-place operations "mul_", "add_" to update target
                # params, as opposed to "mul" and "add", which would make new tensors.
                p_targ.data.mul_(polyak)
                p_targ.data.add_((1 - polyak) * p.data)

        # now replace some part of the replay buffer with rollouts from the policy
        def pipolicy(s, i):
            return ac.act(s[:, 3:], i, deterministic=False)

        def pipolicy_det(s, i):
            return ac.act(s[:, 3:], i, deterministic=True)

        if (step % steps_per_rollout) == 0:
            writer.add_scalar("loss/q", loss_q, step)
            writer.add_scalar("loss/pi", loss_pi, step)

            # collect 1000 samples, rolling out 100 times from a random state
            with torch.no_grad():
                # sample starting state from frozen batch data
                #fsample = torch.randint(0, frozen, (250,), device="cuda")

                fsample = torch.randint(0, replay_size, (nrollouts,), device="cuda")

                # filter out replay_done stateskkkkkj
                bad = replay_done[fsample]
                fsample[bad] = torch.randint(0, replay_size, (bad.sum().item(),), device="cuda")
                bad = replay_done[fsample]
                fsample[bad] = torch.randint(0, replay_size, (bad.sum().item(),), device="cuda")

                # first half of samples must come from frozen set
                # the rest from the full replay buffer
                # (which may have a lot of stuck points but should hopefully become better than frozen over time)
                #fsample[:125] %= frozen

                firsts, firsti = replay_s0[fsample], replay_i0[fsample]

                # for robustness, randomly replace 1% of samples in replay with random ones and roll them out
                randrollouts = step < 10000 and randrollouts1 or randrollouts2
                firsts[:randrollouts], firsti[:randrollouts] = env.randstate(randrollouts)

                s0, i0, a, r, sp, ip, done = rollout(env, firsts, firsti, pipolicy, rolloutlen)
                ravg_ = (r*(~done)).sum()
            writer.add_scalar("reward/train", ravg_, step)
            writer.flush()

            if replay_idx >= replay_size:
                replay_idx = frozen
            u = replay_idx
            replay_idx += nrollouts * rolloutlen
            v = replay_idx

            replay_s0[u:v] = s0.view(-1, s0.shape[-1])
            replay_i0[u:v] = i0.view(-1)
            replay_obs1[u:v] = sp.view(-1, sp.shape[-1])[:, 3:]
            replay_i1[u:v] = ip.view(-1)
            replay_a[u:v] = a.view(-1, a.shape[-1])
            replay_r[u:v] = r.view(-1)
            replay_done[u:v] = done.view(-1)
            if (step % graph_interval) == 0:
                fig = plt.figure(figsize=(12,8))
                plt.plot(track[:, 0] + track[:, 3]*TRACK_HALFWIDTH, track[:, 1] - track[:, 2]*TRACK_HALFWIDTH, '-', color='black')
                plt.plot(track[:, 0] - track[:, 3]*TRACK_HALFWIDTH, track[:, 1] + track[:, 2]*TRACK_HALFWIDTH, '-', color='black')
                plt.scatter(s0[:, :, 0].cpu().numpy(), s0[:, :, 1].cpu().numpy(), c=torch.clip(s0[:, :, 3], 2, 7).cpu().numpy())

                # run a test lap
                s0, i0, a, r, sp, ip, done = rollout(env, testlap_state, testlap_idx, pipolicy_det, 400)
                # also add test rollout to replay buffer at the beginning for further sampling
                u = 0
                v = 400
                replay_s0[u:v] = s0.view(-1, s0.shape[-1])
                replay_i0[u:v] = i0.view(-1)
                replay_obs1[u:v] = sp.view(-1, sp.shape[-1])[:, 3:]
                replay_i1[u:v] = ip.view(-1)
                replay_a[u:v] = a.view(-1, a.shape[-1])
                replay_r[u:v] = r.view(-1)
                replay_done[u:v] = done.view(-1)
                writer.add_scalar("reward/testlap", r.sum(), step)

                plt.plot(s0[:, :, 0].cpu(), s0[:, :, 1].cpu(), '--', color="red")
                plt.xlim(2, 17)
                plt.ylim(-10, 0)
                writer.add_figure('rollouts', fig, global_step=step)

            if (step % 10000) == 0:
                checkpoint()

        step += 1
    except KeyboardInterrupt:
        print("interrupted..")
        break

print("saving final checkpoint")
checkpoint()
