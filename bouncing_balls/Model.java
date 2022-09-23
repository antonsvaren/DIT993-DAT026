package bouncing_balls;
import static java.lang.Math.*;

/*
  The physics model.

  This class is where you should implement your bouncing balls model.

  The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.

  @author Simon Robillard

 */

/*
 Sandra Svensson, 9701178981, datavetenskapligt program, gussvesaas@student.gu.se, hours spent: 10
 Martin Engström, 9807273512, (D) datateknik civilingenjör, martine@chalmers.se,
 Hours spent: 14
 Anton Svarén, 9504253098, datavetenskapligt program, gussvaanc@student.se,
 hours spent:11

 "We hereby declare that we have all actively participated in solving every exercise.
All solutions are entirely our own work, without having taken part in other solutions."
 */


/**
* Conservation of momentum: m1u1 +m2u2 = m1v1 + m2v2
 * Conservation of energy: m1u1^2 + m2u2^2 = m1v1^2 + m2v2^2
 * }=> u2 - u1 = v1 - v2
 * v1 = u2 - u1 + v2
 * => m1u1 + m2u2 - m1(u2 - u1 + v2) = m2v2
 * <=> m1u1 + m2u2 - m1u2 + m1u1 - m1v2 = m2v2
 * <=> 2*m1u1 + m2u2 - m1u2 = (m1 + m2)v2
 * v2 = (2*m1u1 + m2u2 - m1u2)/(m1 + m2) =>
 * v1 = u2 - u1 + (2*m1u1 + m2u2 - m1u2)/(m1 + m2)
 * =(2*m1u1 + m2u2 - m1u2 - m1u1 - m2u1 + m1u2 + m2u2)/(m1 + m2)
 * =(m1u1 + 2*m2u2 - m2u1)/(m1 + m2)
 *
 * v2 = (2*m1u1 + m2u2 - m1u2)/(m1 + m2)
 * v1 = (m1u1 + 2*m2u2 - m2u1)/(m1 + m2)*/

class Model {

	double areaWidth, areaHeight;
	Ball [] balls;

	Model(double width, double height) {

		areaWidth = width;
		areaHeight = height;
		// Initialize the model with a few balls
		balls = new Ball[2];
		balls[0] = new Ball(width / 3, height * 0.9, 1.2, 1.6, 0.2);
		balls[1] = new Ball(2 * width / 3, height * 0.7, -0.6, 0.6, 0.3);
	}

	void step(double deltaT) {
		// TODO this method implements one step of simulation with a step deltaT
		final boolean friction = false; // Change to true to turn on friction
		for (Ball b : balls) {
			final double gravity = 9.82;
			// detect collision with the border
			if (b.x < b.radius && b.vx < 0 || b.x > areaWidth - b.radius && b.vx > 0) {
				b.vx *= -1; // change direction of ball
				if(friction){ // Reduce velocity if friction is turned on
					friction_walls(b);
				}
			}
			if (b.y < b.radius && b.vy < 0 || b.y > areaHeight - b.radius && b.vy > 0) {
				b.vy *= -1;
				if(friction){
					friction_floor(b);
				}
			} else{
				// The ball accelerates with gravity when not in contact with floor
				b.vy -= deltaT*gravity;
			}
			// compute new position according to the speed of the ball
			b.x += deltaT * b.vx;
			b.y += deltaT * b.vy;
		}
			double totalEnergy = 0;

			// detect collision with each ball on every other ball once
			for(int i = 0; i < balls.length; i++){
				Ball b = balls[i]; // First ball
				for(int j = i+1; j < balls.length; j++){
					Ball bj = balls[j]; // Second ball

					// Find distance between ball's centers before movement
					double dx = abs(b.x - bj.x);
					double dy = abs(b.y - bj.y);
					double distance = sqrt(pow(dy,2) + pow(dx,2));
					// If there's a collision before movement
					if(distance <= b.radius+bj.radius){
						// Find distance between ball's centers after movement
						double dx_next = abs(b.x + b.vx*deltaT - bj.x - bj.vx*deltaT);
						double dy_next = abs(b.y + b.vy*deltaT - bj.y - bj.vy*deltaT);
						double distance_next = sqrt(pow(dy_next,2) + pow(dx_next,2));

						// If there's a collision after movement as well
						if(distance > distance_next){
							// m = r^2*Pi
							double mb = pow(b.radius,2)* PI;
							double mbj = pow(bj.radius,2)* PI;

							double v = calculateAngle(b.x,b.y,bj.x,bj.y); // Calculate angle
							// Rotate x-axis and calculate velocities in regards to this axis
							Rect bv_r = rotateVelocity(new Rect(b.vx, b.vy),v);
							Rect bjv_r = rotateVelocity(new Rect(bj.vx, bj.vy),v);
							// Collision calculation of vx on both balls
							double bv_r_tmp = (mb*bv_r.x + mbj*2*bjv_r.x - mbj*bv_r.x)/(mb + mbj);
							bjv_r.x = (2*mb*bv_r.x + mbj*bjv_r.x - mb*bjv_r.x)/(mb + mbj);
							bv_r.x = bv_r_tmp;
							// Rotate the x-axis back and recalculate the velocities for original axis
							bv_r = rotateVelocity(bv_r,-v);
							bjv_r = rotateVelocity(bjv_r,-v);
							//Set the new results
							b.vx = bv_r.x;
							b.vy = bv_r.y;
							bj.vx = bjv_r.x;
							bj.vy = bjv_r.y;
						}
					}
				}
				// Proof that the total energy of the system remains somewhat constant
				// Sigma[m*v^2/2 + h*m*g]
				double m = pow(b.radius,2)*PI;
				totalEnergy += m*(pow(b.vx,2) + pow(b.vy,2))/2 + b.y*m*9.82;
			}
		System.out.println(totalEnergy);
	}
	void friction_walls(Ball b){
		// N*k : k < 1 : N ~= vy*vx*m
		double m = pow(b.radius,2)*PI;
		final double k = 0.2;
		double friction_influence = abs(b.vy*b.vx)*m*k;
		if(b.vy > 0){
			b.vy -= friction_influence;
		}else{
			b.vy += friction_influence;
		}
	}

	void friction_floor (Ball b){
		// N*k : k < 1, Simplification N ~= vy*vx*m
		double m = pow(b.radius,2)*PI;
		final double k = 0.2;
		double friction_influence = abs(b.vy*b.vx)*m*k;
		if(b.vx > 0){
			b.vx -= friction_influence;
		}else{
			b.vx += friction_influence;
		}
	}

	Rect rotateVelocity(Rect p, double v){
		// Matrix multiplication to transform (rotate) a vector
		return new Rect(cos(v) * p.x + sin(v) * p.y, -sin(v) * p.x + cos(v) * p.y);
	}
	double calculateAngle(double x1, double y1, double x2, double y2){
		double a = x1-x2;
		double b = y1-y2;
		return atan(b/a);
	}
	static class Rect{
		double x,y;
		Rect(double x, double y){
			this.x = x;
			this.y = y;
		}
	}

	/**
	 * Simple inner class describing balls.
	 */
	static class Ball {
		
		Ball(double x, double y, double vx, double vy, double r) {
			this.x = x;
			this.y = y;
			this.vx = vx;
			this.vy = vy;
			this.radius = r;
		}

		/**
		 * Position, speed, and radius of the ball. You may wish to add other attributes.
		 */
		double x, y, vx, vy, radius;
	}
}

// Self-check passed!