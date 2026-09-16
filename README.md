# WRO 2026 Future Engineers Superiores
<p align="center">
  <img src="others/banner.png" alt="SUPERIORES" width="100%" style="border-radius: 20px;">
</p>

##  Social Media

<p align="center">
  <a href="https://www.facebook.com/share/19RiiDTZch/">
    <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/facebook/facebook-original.svg" width="45" alt="Facebook">
  </a>
  &nbsp;&nbsp;&nbsp;
  <a href="https://www.instagram.com/losgrisesrt/">
    <img src="https://cdn.jsdelivr.net/gh/simple-icons/simple-icons/icons/instagram.svg" width="45" alt="Instagram">
  </a>
  &nbsp;&nbsp;&nbsp;
  <a href="https://www.tiktok.com/@losgrisesrobotics">
    <img src="https://cdn.jsdelivr.net/gh/simple-icons/simple-icons/icons/tiktok.svg" width="45" alt="TikTok">
  </a>
</p>

## Team Members

#### Team photo

<p align="center">
  <img src="team photos/team_photo.png" alt="Foto de equipo" width="600">
</p>

<table align="center">
  <tr>
    <th colspan="2" align="left">
      Eduardo Alvarado González — Coach & Founder
    </th>
  </tr>
  <tr>
    <td width="260">
      <img src="team photos/Coach_Eduardo.png" width="220">
    </td>
    <td>
      <b>Age:</b> 40<br><br>
      I founded <b>Los Grises Superiores</b> in 2014 with the goal of creating a team where students could learn engineering through real competition experience. Over the years we have participated in multiple WRO and TMR events, reaching both national and international stages. This season I mainly support the team in project organization, technical guidance, and helping the students improve their engineering process during development.
    </td>
  </tr>
</table>

---

<table align="center">
  <tr>
    <th colspan="2" align="left">
      Christopher Pérez Cortés — Programming & Electronics
    </th>
  </tr>
  <tr>
    <td width="260">
      <img src="team photos/Christopher.jpeg" width="220">
    </td>
    <td>
      <b>Age:</b> 14<br><br>
      I joined the robotics club this year after taking an intensive robotics course, and since then I have been learning a lot about electronics, Arduino programming, and 3D design in Onshape. In the team I mainly work on the robot code, sensor integration, and electronics. WRO 2026 is my first international robotics competition, so this season has been a big learning experience for me.
    </td>
  </tr>
</table>

---

<table align="center">
  <tr>
    <th colspan="2" align="left">
      Bárbara Daiana García Balboa — Design & Assembly
    </th>
  </tr>
  <tr>
    <td width="260">
      <img src="team photos/Barbara.jpeg" width="220">
    </td>
    <td>
      <b>Age:</b> 13<br><br>
      I joined the robotics club a few months ago after completing two robotics courses. My main role in the team is helping with the mechanical design, chassis assembly, and testing different structural ideas for the robot. This is my first robotics competition, so I have been learning how the engineering and competition process works while building the project together as a team.
    </td>
  </tr>
</table>


##  Track Challenges

* **Open Challenge:** Complete 3 full laps of the track in the shortest time possible while maintaining lane alignment using sensors (distance and vision).
* **Obstacle Challenge:** Navigate the track and dodge colored pillars in real time using computer vision (camera):
  * **Red Pillars:** Dodge by passing on the right.
  * **Green Pillars:** Dodge by passing on the left.
* **Autonomous Parking:** Detect the designated parking space and perform an automatic parallel parking maneuver upon completion.

## Project Videos

The following videos show the vehicle driving autonomously during both WRO Future Engineers 2026 challenges. Additional video documentation is available in the [video/](video/) folder.

<table>
  <tr>
    <th>Open Challenge</th>
    <th>Obstacle Challenge</th>
  </tr>
  <tr>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=diX7vBUeAKw">
        <img src="https://img.youtube.com/vi/diX7vBUeAKw/hqdefault.jpg" width="400">
      </a>
    </td>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=LjnSCSj2Bsk">
        <img src="https://img.youtube.com/vi/LjnSCSj2Bsk/hqdefault.jpg" width="400">
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=diX7vBUeAKw">Watch on YouTube</a>
    </td>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=LjnSCSj2Bsk">Watch on YouTube</a>
    </td>
  </tr>
</table>

## Project Overview & Abstract

This project was developed as the evolution of our 2025 WRO Future Engineers robot, which successfully reached the international final. Based on the experience gained during that season, the entire vehicle was redesigned for 2026 using a fully 3D-printed chassis, an OpenMV H7 camera, and an Arduino Nano-based control system to improve stability, adaptability, and overall performance.

The robot was built to autonomously complete the two Future Engineers challenges established by the WRO 2026 rules. During the Open Challenge, the vehicle must complete three laps while adapting to randomized track conditions such as driving direction, starting position, and corridor width. In the Obstacle Challenge, the robot must detect colored traffic pillars, avoid them from the correct side, and finally perform a parallel parking maneuver inside the designated parking zone.

Throughout the development process, the team continuously tested and improved both the mechanical and software systems in order to adapt the robot to the new 2026 rule changes and achieve more reliable autonomous behavior.

### Chassis Design & Iteration 

Our first robot designs were based on a LEGO Mindstorms EV3 chassis. While it was easy to build and modify, we noticed several limitations during testing, especially related to weight, space distribution, and steering precision. The EV3 structure was also too restrictive for the type of custom design we wanted to achieve for the 2026 season.

Because of this, we decided to redesign the entire chassis using fully 3D-printed parts designed in Onshape. Our main goal was to create a lighter and more compact structure while still following the WRO size and weight regulations. We also wanted to improve stability, steering performance, and sensor placement.

Throughout the season, we tested different chassis versions. The first prototype used a flat structure, but during steering tests we discovered that the chassis flexed when the servo applied force, causing inaccurate steering movements. To improve rigidity, we redesigned the structure by adding reinforced supports and increasing the wall thickness in critical areas.

Later, we developed a third version focused on improving cornering performance in the new 600 mm corridor introduced in the 2026 rules. We reduced the overall length of the robot and relocated some electronic components to lower the center of gravity and improve balance while turning.

During the final reprint of the chassis using grey PLA filament, we encountered an unexpected problem: the drivetrain became stuck even though the motor was working correctly. After inspecting the structure, we realized that the new filament produced slightly tighter tolerances, causing friction around the rear axle housing. To solve this, we manually sanded the affected area until the shaft rotated smoothly again. After this issue, we started checking all moving parts after every print before assembling the robot.

