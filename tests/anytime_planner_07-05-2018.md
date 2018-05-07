# First trial with node anytime_planner

Seems that loops the "pick n' place operation". 

```sh
charlie@Asgard:~/catkin_ws$ rosrun planner anytime_planner 

	>> WorkCell: /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml
	>> Found device: UR1
	>> Red Ball created and added succesfully to the WorkCell.

Initial and Final configurations checked. <=========> Collision Free


	>> Path's length: 20
	>> Computation time: 0.026 seconds.

	>> Trajectory duration: 2 seconds.
	>> Trajectory length: 191 nodes.
	>> Saved with time steps of 0.01 seconds.

Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_interpolated.txt

CurrentQ = Q[6]{-1.04476, -1.78024, -2.2401, -0.781637, 1.61504, 0.0356452}
Collision status = FALSE
[ INFO] [1525691549.120570233]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.154026805]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.187131965]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.223380187]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.253776962]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.288538802]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.320570424]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.355489365]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.387145034]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.423955232]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.453786643]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.489030011]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.520556678]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.554688206]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.587148329]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.620673073]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.653869359]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.689253765]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.720575239]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.754819137]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.787232814]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.820879692]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.853900778]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.888174077]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.920506028]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.956658909]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691549.987190257]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.023878359]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.053871460]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.087487376]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.120559653]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.157365499]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.187196093]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.222735300]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.253906600]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.288869625]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.320599776]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.354198330]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.387344012]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.423938470]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.454071052]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.487578371]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.520876967]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.556743660]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.587535378]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.620830216]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.654081458]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.688452951]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.720836773]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.754385896]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.787448315]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.824754379]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.854128478]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.888876888]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.920950609]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.955810813]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691550.987357519]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.023239520]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.054222560]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.089140390]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.120927460]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.154899001]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.187552277]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.221098541]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.254231224]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.287918881]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.320980783]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.354998055]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.387600846]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.423716389]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.454185444]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.490922743]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.521044592]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.557156887]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.587621314]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.621975470]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.654349730]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.690895972]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.721076726]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.754356923]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.787441826]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.823613959]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.854155676]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.890488208]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.921135217]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.956725274]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691551.987772988]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.024071766]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.054171496]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.089126902]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.121133145]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.156107399]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.187533960]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.221477556]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.254454675]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.288214006]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.321202702]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.355033658]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.387960128]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.421318578]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.454510515]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.488073569]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.521267808]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.557367964]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.587980572]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.622387389]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.654663723]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.688620657]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.721311163]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.756341359]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.787983524]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.821955107]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.854317929]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.888988448]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.921005046]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.956719047]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691552.987656590]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.021377535]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.054750403]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.090490591]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.120966158]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.156254455]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.187642194]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.221586820]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.254358142]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.289783370]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.321037857]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.354977579]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.387753306]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.421609995]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.454835298]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.488166557]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.521513024]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.555329689]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.587724643]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.623389523]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.654907419]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.690148338]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.721524304]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.756659816]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.787802791]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.821515426]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.854464936]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.888568058]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.921072071]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.955371148]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691553.988258175]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.023140242]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.054967580]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.088454578]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.121762747]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.157401938]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.188203320]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.221777135]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.254890499]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.290770319]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.321689841]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.357248400]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.388481882]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.421862831]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.454543252]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.488826968]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.521821688]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.555474477]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.588523860]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.621944919]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.655301173]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.689459391]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.721912302]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.756921712]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.788126826]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.822537189]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.855142371]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.889843739]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.921859540]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.956879633]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691554.988482746]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.021939200]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.055212253]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.091357237]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.121949653]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.157568363]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.188283845]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.222037900]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.254952075]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.289580062]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.321347980]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.356541196]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.388043914]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.423144005]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.454690296]: CAROS_MOVE_SERVO_Q RESPONSE: 1
Collision status = FALSE
[ INFO] [1525691555.492890091]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.522114133]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.556669504]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.588610142]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.624662650]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.655332236]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.688985394]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.722186646]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.757735363]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.788574551]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.822380470]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.855452132]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.891155459]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.922208004]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.957600287]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691555.988765650]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.022212386]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.055441044]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.092038339]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.122279741]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.155953156]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.188872980]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.226517840]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.255481086]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.288914967]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.321502818]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.356400599]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.388184096]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.422650845]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.455361318]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.489206425]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.522492737]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.557858576]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.588936858]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.626094549]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.655462207]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.690305041]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.722250588]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.758570848]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.789028159]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.824007008]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.855000869]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.890622582]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.922424501]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.958745889]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691556.989159717]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.024921857]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.055061989]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.090235336]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.122547397]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.155391468]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.188289421]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.224661460]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.254973583]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.289860939]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.322659399]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.355600150]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.389185282]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.425731711]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.455758173]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.489852705]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.522603012]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.558142914]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.589329114]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.623061575]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.655807285]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.689447467]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.722541218]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.760674418]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.788958781]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.826128383]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.855899822]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.892350509]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.922682399]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.957463796]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691557.989231334]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.022616526]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.055877074]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.089519232]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.122657818]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.156012712]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.188558431]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.225891678]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.255168552]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.292119055]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.321816617]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.360148487]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.388530431]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.431309915]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.455232160]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.492138739]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.522825097]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.559146298]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.588518290]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.626869659]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.655245072]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.690842424]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.722771289]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.757184946]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.789344198]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.824676000]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.855448626]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.889661288]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.921978072]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.959654257]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691558.989496448]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.024888628]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.056234727]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.090472006]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.122963548]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.158684528]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.189594445]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.224370829]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.255282052]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.289282168]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.321936880]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.359088350]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.388812791]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.426813750]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.455266374]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.489806849]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.521965025]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.557444869]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.588944406]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.623603880]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.656213197]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.690583406]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.723076410]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.757899874]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.789579389]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.825291405]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.856271678]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.891787786]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.922233226]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.958267243]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691559.988736569]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.027910693]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.056445416]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.093336433]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.122156768]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.160402137]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.189930578]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.227147926]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.256473186]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.292722632]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.323185064]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.359526408]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.389808131]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.424351549]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.456452610]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.491219169]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.523263127]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.559905929]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.589826683]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.626483462]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.655679185]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.693369167]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.723246349]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.760531135]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.789812701]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.827092319]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.856512183]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.893786173]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.922414785]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.960268832]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691560.988866162]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.027579144]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.056600323]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.091183693]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.123502456]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.158647919]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.189799015]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.223901014]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.256618143]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.290941045]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.322208101]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.359076948]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.388903126]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.423819034]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.455619843]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.491074527]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.522295622]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.560488210]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.590183874]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.623518252]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.656745945]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.691915329]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.723457908]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.758225051]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.789218817]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.826176404]: CAROS_MOVE_SERVO_Q RESPONSE: 1
Collision status = FALSE
[ INFO] [1525691561.856844372]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.894134445]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.923629548]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.958322692]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691561.990123700]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.026410433]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.056739168]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.091273267]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.123590431]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.159756010]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.190308844]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.225581862]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.257035592]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.292652371]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.322363489]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.360712457]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.390278331]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.426461448]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.456971015]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.490880641]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.523684454]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.559613470]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.590298752]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.625400195]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.657228382]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.696435071]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.723674961]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.759887309]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.790400543]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.826576310]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.856898784]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.892468837]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.923655851]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.958524832]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691562.990472895]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.024789048]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.057248207]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.092966685]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.123882827]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.159232093]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.190380481]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.224259135]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.256073138]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.292960413]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.322767287]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.362112875]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.390478943]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.425659590]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.455901104]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.493243273]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.522629408]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.559037211]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.590700308]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.624882981]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.657170862]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.691903939]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.723930318]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.759411093]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.790654855]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.827706122]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.857340728]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.893983660]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.924125947]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.960591321]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691563.990534752]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.024421653]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.057393258]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.094163043]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.124062088]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.161112734]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.190721096]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.228736680]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.257430428]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.295444389]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.324257221]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.360250017]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.391118414]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.427471525]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.457519986]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.492417838]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.522799910]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.559896513]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.589618293]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.631741788]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.657534799]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.692465912]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.723458115]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.761308039]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.790726776]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.828378144]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.856105699]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.892912094]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.922780717]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.959389210]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691564.989458342]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.031374847]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.056170778]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.095265753]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.123081592]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.160534859]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.190726529]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.226380586]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.258164684]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.294369310]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.324255829]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.362563758]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.390990971]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.424898772]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.457701006]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.494139875]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.524585446]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.559893059]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.590955416]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.624044903]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.656446920]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.694465020]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.724662334]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.761532587]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.789803016]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.826960947]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.858217145]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.895789239]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.922946511]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.962521498]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691565.990500468]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.025866437]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.057877550]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.094922181]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.124457376]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.160093967]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.189663767]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.226152968]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.257916969]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.291519938]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.323372339]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.358676788]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.390374447]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.428799417]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.458181411]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.493013476]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.524780756]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.560488229]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.590669403]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.628166209]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.656365543]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.693578409]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.724971769]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.762041058]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.791871926]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.826427860]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.858171867]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.896848400]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.925050638]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.959264348]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691566.990227099]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.026110396]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.058413188]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.093565785]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.123831020]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.163524819]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.191694608]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.229450066]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.258155369]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.296848302]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.325242238]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.360946391]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.390053472]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.427876333]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.457857909]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.495003197]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.525097537]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.563139886]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.591807550]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.625737492]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.656518782]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.694700650]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.723365674]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.759315903]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.791375863]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.825850648]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.858287255]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.892780428]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.923337958]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.959322818]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691567.990101515]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.025464064]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.056560004]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.095191143]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.125057732]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.161082185]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.191794468]: CAROS_MOVE_SERVO_Q RESPONSE: 1
Collision status = FALSE
[ INFO] [1525691568.228979329]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.258243999]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.295100918]: CAROS_MOVE_SERVO_Q RESPONSE: 1
[ INFO] [1525691568.325014992]: CAROS_MOVE_SERVO_Q RESPONSE: 1


			   .....

```