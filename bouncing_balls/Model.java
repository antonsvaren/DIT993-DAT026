package bouncing_balls;
import java.lang.Math;

import static java.lang.Math.*;
/**
 * Martin: 5h
 */

/**
 * The physics model.
 * 
 * This class is where you should implement your bouncing balls model.
 * 
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 * 
 * @author Simon Robillard
 *
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
 * v1 = (m1u1 + 2*m2u2 - m2u1)/(m1 + m2)
 * v1 = hastighetsvektor efter kollision, m1 = massa boll1, m2= massa boll2, u1 = hastighet boll1, u2= hastighet boll2*/

class Model {

	double areaWidth, areaHeight;
	
	Ball [] balls;

	Model(double width, double height) {
		areaWidth = width;
		areaHeight = height;
		
		// Initialize the model with a few balls
		balls = new Ball[2];
		/*
		balls[0] = new Ball(width / 3, height * 0.9, 1.2, 1.6, 0.2);
		balls[1] = new Ball(2 * width / 3, height * 0.7, -0.6, 0.6, 0.3);*/
		// Balls colliding horizontally:
		balls[0] = new Ball(0.2, height * 0.9, 1.2, 0, 0.2);
		balls[1] = new Ball(width - 0.3, height * 0.9, -0.6, 0, 0.3);
		//balls[2] = new Ball(2, height * 0.9, 0.8, 0, 0.25);
	}

	void step(double deltaT) {
		// TODO this method implements one step of simulation with a step deltaT
		for (Ball b : balls) {
			// detect collision with the border
			if (b.x < b.radius || b.x > areaWidth - b.radius) {
				b.vx *= -1; // change direction of ball
			}
			if (b.y < b.radius || b.y > areaHeight - b.radius) {
				b.vy *= -1;
			}
			
			// compute new position according to the speed of the ball
			b.x += deltaT * b.vx;
			b.y += deltaT * b.vy;
		}

			// detect collision with each ball on every other ball once
			for(int i = 0; i < balls.length; i++){
				Ball b = balls[i];
				for(int j = i+1; j < balls.length; j++){
					Ball bj = balls[j];


					// Find distance between ball's center before movement
					double dx = abs(b.x - bj.x);
					double dy = abs(b.y - bj.y);
					double distance = sqrt(pow(dy,2) + pow(dx,2));

					// If there's a collision before movement
					if(distance <= b.radius+bj.radius){
						// Find distance between ball's center after movement
						double dx_next = abs(b.x + b.vx*deltaT - bj.x - bj.vx*deltaT);
						double dy_next = abs(b.y + b.vy*deltaT - bj.y - bj.vy*deltaT);
						double distance_next = sqrt(pow(dy_next,2) + pow(dx_next,2));

						// If there's a collision after movement
						if(distance > distance_next){
							System.out.println("COLLISION");
							// m = r^2*Pi
							double mb = pow(b.radius,2)* PI;
							double mbj = pow(bj.radius,2)* PI;

							/* 	v1 = (m1u1 + 2*m2u2 - m2u1)/(m1 + m2)
								v2 = (2*m1u1 + m2u2 - m1u2)/(m1 + m2)*/
							double bv = (mb*b.vx + mbj*2*bj.vx - mbj*b.vx)/(mb + mbj);
							double bjv = (2*mb*b.vx + mbj*bj.vx - mb*bj.vx)/(mb + mbj);
							b.vx = bv;
							bj.vx = bjv;
						}
					}

				}
			}
	}

	Polar rectToPolar(double x, double y){
		double r = sqrt(pow(x,2)+pow(y,2)); // Length
		double v = atan2(y,x); // Angle
		return new Polar(r,v);
	}
	Rect polarToRect(Polar p){
		double x = cos(p.v)*p.r;
		double y = sin(p.v)*p.r;
		return new Rect(x,y);
	}
	class Rect{
		double x,y;
		Rect(double x, double y){
			this.x = x;
			this.y = y;
		}
	}
	class Polar {
		double r,v;
		Polar(double r, double v){
			this.r = r;
			this.v = v;
		}
	}

	/**
	 * Simple inner class describing balls.
	 */
	class Ball {
		
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
