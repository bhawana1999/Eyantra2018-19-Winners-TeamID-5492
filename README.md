# Eyantra2018-19 : [Winners TeamID #5492](http://www.e-yantra.org/eyrc)
Includes ROS, image processing and Proportional Integral Derivative

This projects imitates the behaviour of pollinator bee. As the bee goes from flower to flower and pollinates them, here the drone(PlutoX) takes off from beehive, goes to each flower, pollinates them and lands back to the  beehive.


<img src="https://github.com/sona-19/Eyantra2018-19/blob/master/2.jpeg" width="250">

<b>Design of Flower</b>  :- Design of Flower needs to be made such that it increases the chances of pollination and at the same time decreases the chances of collision. The flower is made in a domb shape by soldering wire placed very close to each other, so that it gets pollinated, even with the slightest touch.

<b>Bee Stinger Design</b>  :- The stinger is made by strips of aluminium foil placed below the drone. 

Design of flower and Stinger :-

<img src ="https://github.com/sona-19/Eyantra2018-19-Winners-TeamID-5492/blob/master/images_eyrc2018/IMG_7756.JPG" width = "400">

<b>Pollination</b>

<img src="https://github.com/sona-19/Eyantra2018-19/blob/master/pollination.jpeg" width="340">
<img src= "https://github.com/sona-19/Eyantra2018-19-Winners-TeamID-5492/blob/master/images_eyrc2018/HGJ_0541.JPG"  width="340">

Major challenges in this theme:

1. PID tuning
2. Waypoint Navigation
3. Avoiding Collision with flower
4. Increasing probability of pollination
5. Landing of drone


<b>Various Algorithms Designed to overcome the above challenges</b>

1.<b>Landing of Drone</b>:- The algorithm that made the drone land involves giving 3 waypoints at various heights above the beehive on which the drone was supposed to land. The thresholds given for each of those points were different. For example the threshold for the topmost point was more than that of the point just below it. Similarly, the point just above the beehive had minimum threshold. This resulted in formation of a cone that facilitated the landing of the drone. 


2.<b>Avoiding Collision with flower</b>:- In order to avoid collision betwwen the drone and the flower, the midpoint algorithm was used. In this algorithm, certain points were given between two flowers so that the drone may not collide with the flowers due to sudden approaching or sudden decrease in the throttle. The midpoint algorithm calculates the midpoint coordinates between the two flowers and give those as the x and y coordinate to the drone before it reaches the second flower. But the z coordinate was given as the maximum height of the two flowers as it was observed that mainly the collisions were due to sudden increase or decrease in the throttle. But in the case of midpoint coordinate, the threshold of x and y coordinate was more than the z coordinate because the objective was to avoid sudden increse or decrease in the throttle. Although giving the same threshold in x and y coordinate would not have created any problem but it would have cost us a lot of time.

3.<b>Increasing probability of pollination</b>:- It was observed that sometimes drone gets stabilized at some point near to the flower and doesn't move further as it has not pollinated the flower. This was due to several reasons like movement of the camera that changed all the coodinates or noting down the wrong coordinates. To resolve this problem, we introduced a timer and a backup algorithm for pollination. For example if the flower was not pollinated in 15 seconds after the last point was reached then the drone start moving on several different points for 5 seconds each and these points were - (x, y-2), (x, y+2), (x-2, y-2) and soon. But as soon as the flower gets pollinated during this algorithm, the drone moves on to the next flower. And if the flower does not get pollinated even after this algorithm, the flower is skipped and the drone  moves to the next point.





[Project Video](https://youtu.be/_y-xsZQv6v4)

[eYRC Video](https://www.youtube.com/watch?v=d3jKvnjwe4E&feature=youtu.be)
