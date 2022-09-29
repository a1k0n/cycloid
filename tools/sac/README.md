# soft actor-critic implementation for diyrobocars

trains 2x *Q* networks and one &pi; network.

the network architecture has an embedding layer like a language model has; one
*d*-dim vector for each point on the track. Track data is a side input
containing x, y of centerline and unit vector pointing forward along the track.
The Circuit Launch track this was initially trained on has 315 points,
subdivided roughly equidistantly from an SVG drawing.

So each little spot on the track is kind of like a token in a language model,
so that the network can learn the optimal way through the track at that
particular spot.

The embedding is added to the local state [x, y in local track frame, sin/cos
of direction in local track frame, velocity, yaw rate] projected through a
linear layer.

```
 [state vector]                              [action (Q network only)]
----------------                                 ---------------
| linear 6 x H |      [track embedding, dim=H]   | linear 2 x H|
       |                        |                   |
       `----------(+)-----------'-------------(+)---'
                   |
            |Mish activation|
                   |
            |   HxH linear  |
                   |
            |Mish activation|
                   |
         .-----------------------------------.
         |                |                  |
   | linear Hx2 |   | linear Hx2 |     | linear Hx1 |
      action mu     action logstd          Q value
  |-------pi network only--------|  |-Q networks only-|
```

The *Q* network has 128 hidden dimensions; the &pi; network has 16.

So basically, I just add the track embedding to the projection of the state
(/action as applicable), mish activation, run it through a fully-connected
layer, another mish activation, then a linear projection to get actions or Q
values. The architecture is extremely simple because the control policy is
locally extremely simple; the embedding layer contains all the information on
the best racing line.

There is an alternative, more general way to do this that wouldn't be
map-specific, which is to feed the future curvature information into the
network as an input.
