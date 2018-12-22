#include <math.h>
#include <stdio.h>
#include "drive/trajtrack.h"

TrajectoryTracker::TrajectoryTracker() {
  n_pts_ = 0;
  pts_ = NULL;
}

TrajectoryTracker::~TrajectoryTracker() {
  delete[] pts_;
}

bool TrajectoryTracker::LoadTrack(const char *fname) {
  if (pts_ != NULL) {
    delete[] pts_;
  }

  FILE *fp = fopen(fname, "r");
  if (!fp) {
    perror(fname);
    return false;
  }

  if (fscanf(fp, "%d\n", &n_pts_) != 1) {
    fprintf(stderr, "failed loading %s\n", fname);
    fclose(fp);
    n_pts_ = 0;
    return false;
  }

  pts_ = new TrajectoryPoint[n_pts_];
  for (int i = 0; i < n_pts_; i++) {
    if (fscanf(fp, "%f %f %f %f %f\n",
        &pts_[i].x, &pts_[i].y,
        &pts_[i].nx, &pts_[i].ny,
        &pts_[i].k) != 5) {
      fprintf(stderr, "failed to load waypoint %d\n", i);
      fclose(fp);
      n_pts_ = 0;
      delete[] pts_;
      return false;
    }
  }

  fprintf(stderr, "*** loaded %d waypoints\n", n_pts_);

  fclose(fp);
  return true;
}

bool TrajectoryTracker::GetTarget(float x, float y, int lookahead,
    float *closestx, float *closesty,
    float *normx, float *normy,
    float *kappa, float *lookahead_kappa, float *dist) {

  if (n_pts_ == 0) {
    return false;
  }

  int mini = 0;
  float mind = 1e12;

  for (int i = 0; i < n_pts_; i++) 
  {
    const TrajectoryPoint &p = pts_[i];
    float dist = (p.x - x)*(p.x - x) + (p.y - y)*(p.y - y);
    if (dist < mind) 
    {
      mind = dist;
      mini = i;
    }
  }

/*

*/

//So the idea is, we find the first local maxima of the curvature of the track ahead of us. The reason why 
//we want the first local maxima is because the points under consideration might include 2 points where the kappa is high (like in a chicane). 
//We only concern ourselves with the first instance of max kappa.
  float del_k, last_del_k, braking_dist[2], pos_x[2], pos_y[2];
  float max_k[2]; //upto 2 local maximas. Why 2? because it is highly unlikely that an optimized trajectory would contain 3 local maximas in succession
                  //so close to each other and so drastically different that premature braking would be required for the 3rd local maxima. Thats just bad optimisation IMO
  uint8_t count = 0; //sentinel variable
  float out_k, out_d; //these will be the final outputs.

  for (int i = mini; i < mini + n_pts_ ; i++) //circle around from the current target position.
  {
    int a = i%n_pts_ ; // prevent memory issues.
    int b = (i-1)%n_pts_ ;
    const TrajectoryPoint &p1 = pts_[a], &p2 = pts_[b]; 
    del_k = p1.k - p2.k ; //derivative of curvature. (not really, its just change in curvature but its proportional to the derivative)
    if(i == mini) //first pass. establish last_del_k
    {
      last_del_k = del_k;
    }
    if(i != mini)
    {
      if(del_k*last_del_k < 0) // the sign of the derivative of the curvature changes around the sharp turns. therefore if the product of successive derivatives is negative, it indicates a sharp turn
      {
        max_k[count] = p2.k; //store value of max_k,distance and break the loop.
        braking_dist[count] = sqrt((p2.x - x)*(p2.x - x) + (p2.y - y)*(p2.y - y)); //get value of approximate distance to that point 
        //I say approximate because it is the straight line distance and not the distance along the trajectory (while that would be ideal, it would take a while (for me) to figure out) 
        pos_x[count] = p2.x;
        pos_y[count] = p2.y; //store coordinates
        count++;
      }
      if(count>=2) //2 points found
      {
        break;
      }
    }
  }

  out_k = max_k[0];
  out_d = braking_dist[0];

  float gap_inverse = 1/sqrt( (pos_x[1] - pos_x[0])*(pos_x[1] - pos_x[0]) + (pos_y[1] - pos_y[0])*(pos_y[1] - pos_y[0]) ); //distance between the 2 points where local maximas occur.
  float k_dash_inverse = 1/sqrt( fabs(max_k[0]*max_k[1]) );
  float k1_inverse = 1/fabs(max_k[0]);
  float condition = gap_inverse*(k1_inverse - k_dash_inverse) ;
  if(condition > 1) //this condition was determined mathematically. Images for proof will be in pull request.
  {
    out_k = max_k[1];
    out_d = braking_dist[1];
  }
  //intuitive explanation : the farther away the points are, the less important the second maxima (gap inverse). if the second local maxima has a curvature larger than the first,
  // the quantity "condition" becomes larger as k_dash_inverse is inversely proportional to second maxima's magnitude and therefore resulting in a larger net result.
  // if the first local maxima has the same curvature as the second, the value of condition becomes 0 and negative if the curvature of the first maxima is greater than that of the second maxima.

  // int li = (mini + lookahead) % n_pts_; //didn't seem like this line was doing anything useful so I commented it.

  const TrajectoryPoint &p = pts_[mini];
  *closestx = p.x;
  *closesty = p.y;
  *normx = p.nx;
  *normy = p.ny;
  *kappa = p.k;
  *lookahead_kappa = out_k;
  *dist = out_d; //will need this later 
  // printf("i %d pt %f %f norm %f %f k %f\n", mini, p.x, p.y, p.nx, p.ny, p.k);

  return true;
}
