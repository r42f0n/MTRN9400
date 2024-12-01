java c
MTRN9400   Control   of   Robotic   Systems
ASSIGNMENT   2 -   T3   2024
Overview   of the   Assignment
This   assignment   focuses   on   the   control   of   quadcopters   and   is   worth   20%   of   your   total   mark   in this   course.The   due   date   is   3:00am   on   Tuesday   Week   12.    This   deadline   is   strict   and   will   not   be      extended.   Marks   will   be   returned   within   2   weeks   after   the   submission   deadline.    You   will   complete   this   assignment   individually.
Learning   Outcomes
This   assignment   specifically   targets   the   following   learning   outcome:
•      LO1:   Stability   analysis   and   control   of nonlinear   systems.
•      LO2:   Understanding   different   classes   of   controllers   and   apply   them   to   robotic   systems.
Assignment   SpecificationThis   assignment   is   split   into   four   parts.    This   document   will   explain   what   you   are   required   to   do in each part,   and the   contribution   each   part   will   make   to   your   overall   assignment   mark.   You   are   given   MATLAB   files   that   will   be   used   for   the   simulation   parts   of the   assignment.
In   this   assignment,   you   will   design   several   controllers   for   a   quadcopter   that   flies   in   1D   and   2D   spaces.   The   marks   allocated   to   each   part   of the   assignment   are   as   follows   (total   20   marks):
•      Part   1   (4   marks):   PD   control   in   1D
•      Part   2   (5   marks):   Sliding   mode   control   in   1D
•      Part   3   (3.5   marks):   Adaptive   control   in   1D
•      Part   4   (5.5   marks):   2D   control
•      Presentation   (2 marks):   The   criteria   include   good   spelling   and   grammar, appropriate   tech-   nical   language, logical   structure   including   using   headings   and   other   conventions,   appropri-   ate   graphical   and   tabular   presentation,   caption   for   graphics   and   tables,   and   appropriate spacing   that   aids   readability.
1          Part   1   (4   marks)
In   this   part,   you   will   control   the   motion   of a   quadcopter   flying   in   the   z-direction.   As   explained   in   the   lecture,   the   dynamic   model   of a   quadcopter   in   a   1D   space   is
z   =   −g +      m/u1                                                                                                                           (1)where   u1      is   the   thrust   generated   by   the   drone   motors,    g    =    9.81    m/s2       is      the      gravitational acceleration,   and   m   is   the   quadcopter   mass.   If the   mass   of the   quadcopter   is   known,   a   control   law   u1    can   be   designed   as
u1   (t) = m   (¨(z)d(t) − kpe(t) − kd˙(e)(t) +   g),                                                                                                                           (2)
where   zd(t)   is   the   desired   height   of   the   quadcopter   at   time   t,   kp      and   kd      are   the   control   gains,   and
e(t) = z(t)   − zd(t).                                                                                                                                                                              (3)
Then   the   closed-loop   system   is   obtained   from   (1)-(3)   as
¨(e)(t) + kd      ˙(e)(t) + kp      e(t) = 0.                                                                                                                                                      (4)
We   assume   in   this   part   of the   assignment   that   zd    is   a   constant   scalar   and   is   equal   to   1   m.   This   means   that   we   want   to   control   a   quadcopter   to   rise   to   a   height   of   1   meter   and   stays   there.The   controller   in   (2)   is   already   implemented   in   the   provided   MATLAB   code.      Open   the   code   for   Part   1   and   then   open   the   ‘System1D         1.m’   file.   As   shown   below,   the   commented   lines   at   the   top   of   ‘System1D         1.m’   file   (lines   10–14)   explain   how   the   state   variables   are   defined.
10                  %                Inputs   :
11                  %                                 t                         :                  A    scalar    containing    the    current      time
12                  %                         s                         :                  A    2x1    vector    containing    the    current    state         [   z   ;    z         dot   ]
13                  %                Outputs   :
14                  %                                 s         dot         :                  A    2x1      vector    containing         [ds1   ;    ds2   ]
For   example,   z(t)   and   ˙(z)(t)   are   stored   respectively   in   s(1)   and   s(2).   Quadcopter   parameters   are
also   defined   in   lines   20–24.
20             global    QuadParams
21          QuadParams   .gravity                           =    9   .   81;                               %    gravitational    constant
22             QuadParams   .mass                                        =      0   .28;                                       %    mass    of      quadcopter
23             QuadParams   .arm         length    =    0   .   086;                            %    wingspan    of    quadcopter
24          QuadParams   .height                            =    0   .   05;                                  %    height    of      quadcopterThe   desired   trajectory   of   the   quadcopter   is   defined   in   lines   26–31.    The   desired   height,   zd(t),   is stored in   s   des(1),   and   ˙(z)d(t) is stored in   s   des(2).   You   should   not   change   any   of the   above   parts   in your   MATLAB   file.
The   control   law   u1    in   (2)   is   implemented   in   line   48   of   ‘System1D         1.m’   file:
48             u1    =    QuadParams   .mass      *       (z         ddot      −    kp   *e      −      kd   *e         dot      +      QuadParams   .gravity);Run   the   ‘main1D         1.m’   file.   By   executing   this   file,   a   figure   will   be   generated   showing   a   2D   and a   3D   view   of   the   quadcopter   position   for   t   between   0   and   5s   and   also   a   plot   of   z(t).    You   will see   that   z(t)   converges   to   zd      =   1   m.    The   steady   state   error   will   be   printed   on   the   Command Window.(a)    Change   the   control   law   in   line   48   of   the   ‘System1D         1.m’   file   to   the   following   PD   controller:   u1   (t) =   −kpe(t) − kd˙(e)(t),           (5)where   kp      and   kd      are   the   proportional   and   the   derivative   gains,   respectively.      These   gains   are   already   defined   in   lines   46-47   of   the   code.       Do    not   change   their   values    and   run   the   ‘main1D         1.m’代 写MTRN9400 Control of Robotic Systems ASSIGNMENT 2 - T3 2024Matlab
代做程序编程语言   file.You   will   observe   that   z(t)   does   not   converge   to   zd      =    1.    In   your   report,    add   a   screenshot from   the   z-plot   that   clearly   shows   the   value   of   z(t)   at   t   ≈ 5s.    You   can   use   a   ‘Data   Tip’   to get   the   value   of   z(t)   from   the   MATLAB   figure.    Write   the   value   of   e(t)   at   t   ≈   5s   in   your   report.      (1   mark)(b)    Substitute   the   control   law   (5)   into   the   system   model   (1)   and   write   the   closed-loop   system.   Then   obtain   the   error   dynamics   (which   is   the   dynamical   model   that   depends   on   the   error   signal   e(t)   and   its   time   derivatives,   but   is   independent   of   z(t)   and   its   derivatives).   Finally,   write   the   error   dynamics   in   the   state   space   form   and   find   the   equilibrium   points.   You   will   see   that   the   equilibrium   point   is   not   at   the   origin.    Verify   that   the   steady-state   error   you   obtained   in the simulation of Part   1(a)   is   in   fact   an   equilibrium   point   of the   error   dynamic.
(2   marks)
(c)   Find   the   range   of   values   for   kp      and   kd      such   that   the   absolute   value   of   the   steady-state   position   error   (|e(t)|   for   a   large   t)   is   less   than   0.01.   You   should   explain   in   your   report   how you   calculated   these   values.    You   should   analytically   choose   these   values   (not   by   trial   and   error   using   MATLAB)   and   do   not   need   to   add   any   MATLAB   simulation   results   to   your   report   for   this   part; just   a   mathematical   calculation   is   needed   for   this   part.    (1   mark)
2          Part   2   (5   marks)As we observed   in   Part   1,   a   PD   controller   does   not provide   a zero   steady-state   error.    So   we   will   use   a   robust   controller   in   this   part   of the   assignment.    For   this   part,   use   the   MATLAB   files   in   ‘Part   2’   folder.   We   continue   to   assume   in   this   part   that   zd   =   1   m.
Consider   the   following   sliding   mode   surface
S   =   e + a˙(e)                                                                                                                                                                                                (6)
where   a   is   a   positive   constant   scalar.   We   define   the   sliding   mode   control   as   follows:
u1 = m (-a/e + g - p sgn(s))                                                                                                                   (7)   
(a)   Use   the   Lyapunov   candidate   V   =   S2      and   find   all   possible   values   for   a   and   ρ   such   that
both   e(t)   and   ˙(e)(t)   converge   to   zero   as   t   →   ∞   .   You   should   add   your   detailed   stability   proof
in your   report.    (2   marks)
(b)    Implement the control law   (7) in ‘System1D         2’ and choose a and ρ such that e and   ˙(e)   converge   to   zero   within   5   seconds.    The   sliding   mode   controller   is   implemented   in   lines   41–45   of the   ‘System1D   2’   file:
41             rho    =    0;                      %    Change    this    value
42             a    =      0   .3;                                       %    Change    this    value
43             S      =      e      +      a   *e         dot;
44          ud    =    QuadParams   .mass   * (−e         dot/a    +    QuadParams   .gravity    −      rho   *sign(S));
45               u1    =      ud;Add   the   z-plot   figure   from   your   simulation   to   demonstrate   z(t) converges   to   zd.   You   will   get a   full   mark   if   the   oscillation   in   z   is   damped   by   5   seconds   (it   is   OK   if   there   is   no   oscillation at   all)   and   z(5) is between   0.997   and   1.003.    Use   a Data Tip in   your   figure   to   show   the   value   of   z(t)   at   t   = 5   s.    Write   the   values   you   used   for   a   and   ρ   in   your   report.    (2   marks)
Hint:    Use   capital   letter   S   in   your   code   to   define   the   sliding   surface   as   the   lower   case   s   is   already   used   in the   code.    MATLAB   has   an   in-built   function,   sign,   for   the   signum   function.
(c)    So   far,   we   observed   that   a   sliding   mode   controller   performs   better   than   a   PD   controller as   the   steady-state   error   in   the   sliding   mode   controller   converges   to   zero.      We   also   know   that   sliding   mode   control   is   robust   against   system   uncertainties.    So   we   will   assume   in   this part   that   there   is   an   unknown   constant   bias   d   in   the   control   input   which   is   applied   to   the   quadcopter   rotors   as   follows
u1   (t) = ud(t) + d                                                                                                                                                                        (8)
where   ud    is   the   nominal    control   law    defined   as
ud = m (-a/e + g - p sgn(s))                                                                                                                   (9)   This   is   a   common   practical   issue   when   implementing   a   controller   to   a   real   system   as   the   system   actuators   may   not   be   able   to   exactly   generate   the   commanded   forces/torques.    In this   part   of the   assignment,   we   assum   there   is   always   a   bias   in   the   system’s   actuator.   Note   that   ud    in   (9)   is   the   same   as   the   controller   (7)   we   used   in   the   previous   part.
To   implement   this   biased   system   in   MATLAB,   comment   line   45   and   uncomment   line   46   of the   code   as   shown   below:
45       % u1 = ud;46       u1 = ud − 0.1;   

As   can   be   seen   above,   we   assumed   the   unknown   disturbance   is   d   =   −0.1.   Please   note   that   this value is unknown to   us   as   control   engineers   and   therefore   the   controller   ud   does   not   use   this   value.    We   added   this   line   to   the   code   so   that   the   quadcopter   behaves   as   if   there   is   a   bias   in   the   thrust   force   which   is   generated   by   the   rotors.
Choose   a   and   rho   such   that   the   |e(t)|    and   |   ˙(e)(t)|    at   t   =   5   s   are   less   than   0.01.      Only   add   the   values   you   found   for   a   and   rho   in   your   report.      No   explanation   is   needed   for   this   part.Also   no   figure   is   required.   We   will   check   the   values   you   will   provide   and   will   then   give   you a   full   mark   if   the   error   conditions   are   satisfied,   and   zero   marks   if   they   are   not   satisfied.    (1   mark)



         
加QQ：99515681  WX：codinghelp  Email: 99515681@qq.com
