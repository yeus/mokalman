package kalman
  model kalman_discrete
  end kalman_discrete;

  package Examples
    extends Modelica.Icons.Example;

    model rigid_body_states "https://en.wikipedia.org/wiki/Rigid_body_dynamics#Rotation_in_three_dimensions"
      inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity)  annotation(
        Placement(visible = true, transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Body body1(v_0(start = {0.1, 0.2, 0.3}), w_a(start = {0.0, 0.0, 0.3}))  annotation(
        Placement(visible = true, transformation(origin = {30, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {1, 0, 0})  annotation(
        Placement(visible = true, transformation(origin = {-16, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
      connect(fixedTranslation1.frame_b, body1.frame_a) annotation(
        Line(points = {{-6, 26}, {20, 26}, {20, 26}, {20, 26}}, color = {95, 95, 95}));
    
    annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-6, Interval = 0.2));end rigid_body_states;

  model mass_estimate "estimate mass of a body"
      inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity)  annotation(
        Placement(visible = true, transformation(origin = {-64, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
        Placement(visible = true, transformation(origin = {-32, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(m = 1)  annotation(
  
        Placement(visible = true, transformation(origin = {10, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity v annotation(
        Placement(visible = true, transformation(origin = {14, -26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition p annotation(
        Placement(visible = true, transformation(origin = {-8, -26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  blocks.noise_sampled noise_sampled1[3](samplePeriod = dT, variance = {2.0, 2.0, 2.0})  annotation(
        Placement(visible = true, transformation(origin = {26, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine1[3](amplitude = {0, 0, 1.0}, freqHz = {1.0, 1.0, 0.1})  annotation(
        Placement(visible = true, transformation(origin = {-82, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real sigma_u = 0.1;
  parameter Real dT = 2.0;
  parameter Real[2,1] B=[dT * dT * 0.5; dT];
  blocks.kalman kalman(A = [1, dT; 0, 1], B = B, H = [0, 1], Q = B * transpose(B) * sigma_u * sigma_u, R = [0.5], dT = dT, sigma_u = sigma_u)  annotation(
        Placement(visible = true, transformation(origin = {56, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  blocks.noise_sampled noise_sampled2[3](samplePeriod = dT, variance = {2.0, 2.0, 2.0}) annotation(
        Placement(visible = true, transformation(origin = {27, -45}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  equation
      connect(p.r, noise_sampled2.u) annotation(
        Line(points = {{-8, -36}, {-8, -36}, {-8, -46}, {20, -46}, {20, -44}}, color = {0, 0, 127}, thickness = 0.5));
      connect(noise_sampled1[3].y, kalman.z[1]) annotation(
        Line(points = {{36, -70}, {40, -70}, {40, -4}, {46, -4}, {46, -4}, {48, -4}}, color = {0, 0, 127}, thickness = 0.5));
      connect(sine1[3].y, kalman.u[1]) annotation(
        Line(points = {{-70, 8}, {-66, 8}, {-66, -6}, {32, -6}, {32, 2}, {48, 2}, {48, 2}}, color = {0, 0, 127}, thickness = 0.5));
//connect(sine1[3].y, stateSpace1.u[1]) annotation(
//  Line(points = {{-70, 8}, {-50, 8}, {-50, -14}, {60, -14}}, color = {0, 0, 127}));
      connect(body1.frame_a, p.frame_a) annotation(
        Line(points = {{0, 12}, {-8, 12}, {-8, -16}}, color = {95, 95, 95}));
      connect(body1.frame_a, v.frame_a) annotation(
        Line(points = {{0, 12}, {-8, 12}, {-8, -8}, {14, -8}, {14, -16}}, color = {95, 95, 95}));
      connect(force.frame_b, body1.frame_a) annotation(
        Line(points = {{-22, 8}, {-11, 8}, {-11, 12}, {0, 12}}, color = {95, 95, 95}));
      connect(v.v, noise_sampled1.u) annotation(
        Line(points = {{14, -38}, {14, -50}, {-4, -50}, {-4, -70}, {17, -70}}, color = {0, 0, 127}));
      connect(sine1.y, force.force) annotation(
        Line(points = {{-70, 8}, {-44, 8}, {-44, 8}, {-44, 8}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-6, Interval = 0.2),
        Documentation(info = "<html><head></head><body>This example demonstrates estimation of the mass<div>of a body through only velocity measurements</div></body></html>"));
  end mass_estimate;






    model simple_tracking
      inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
        Placement(visible = true, transformation(origin = {-64, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
        Placement(visible = true, transformation(origin = {-32, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Body body1(m = 1) annotation(
        Placement(visible = true, transformation(origin = {10, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity v annotation(
        Placement(visible = true, transformation(origin = {14, -26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition p annotation(
        Placement(visible = true, transformation(origin = {-8, -26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      blocks.noise_sampled noise_sampled1[3](samplePeriod = dT, variance = {2.0, 2.0, 2.0}) annotation(
        Placement(visible = true, transformation(origin = {26, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Sine sine1[3](amplitude = {0, 0, 1.0}, freqHz = {1.0, 1.0, 0.1}) annotation(
        Placement(visible = true, transformation(origin = {-82, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Real sigma_u = 0.1;
      parameter Real dT = 0.1;
      parameter Real[2, 1] B = [dT * dT * 0.5; dT];
      blocks.kalman kalman(A = [1, dT; 0, 1], B = B, H = [1, 0; 0, 1], Q = B * transpose(B) * sigma_u * sigma_u, R = [0.5, 0; 0, 0.5], dT = dT, sigma_u = sigma_u) annotation(
        Placement(visible = true, transformation(origin = {56, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      blocks.noise_sampled noise_sampled2[3](samplePeriod = dT, variance = {2.0, 2.0, 2.0}) annotation(
        Placement(visible = true, transformation(origin = {27, -45}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
    equation
      connect(p.r, noise_sampled2.u) annotation(
        Line(points = {{-8, -36}, {-8, -36}, {-8, -46}, {20, -46}, {20, -44}}, color = {0, 0, 127}, thickness = 0.5));
      connect(noise_sampled2[3].y, kalman.z[1]) annotation(
        Line(points = {{34, -44}, {36, -44}, {36, -4}, {48, -4}, {48, -4}}, color = {0, 0, 127}, thickness = 0.5));
      connect(noise_sampled1[3].y, kalman.z[2]) annotation(
        Line(points = {{36, -70}, {40, -70}, {40, -4}, {46, -4}, {46, -4}, {48, -4}}, color = {0, 0, 127}, thickness = 0.5));
      connect(sine1[3].y, kalman.u[1]) annotation(
        Line(points = {{-70, 8}, {-66, 8}, {-66, -6}, {32, -6}, {32, 2}, {48, 2}, {48, 2}}, color = {0, 0, 127}, thickness = 0.5));
//connect(sine1[3].y, stateSpace1.u[1]) annotation(
//  Line(points = {{-70, 8}, {-50, 8}, {-50, -14}, {60, -14}}, color = {0, 0, 127}));
      connect(body1.frame_a, p.frame_a) annotation(
        Line(points = {{0, 12}, {-8, 12}, {-8, -16}}, color = {95, 95, 95}));
      connect(body1.frame_a, v.frame_a) annotation(
        Line(points = {{0, 12}, {-8, 12}, {-8, -8}, {14, -8}, {14, -16}}, color = {95, 95, 95}));
      connect(force.frame_b, body1.frame_a) annotation(
        Line(points = {{-22, 8}, {-11, 8}, {-11, 12}, {0, 12}}, color = {95, 95, 95}));
      connect(v.v, noise_sampled1.u) annotation(
        Line(points = {{14, -38}, {14, -50}, {-4, -50}, {-4, -70}, {17, -70}}, color = {0, 0, 127}));
      connect(sine1.y, force.force) annotation(
        Line(points = {{-70, 8}, {-44, 8}, {-44, 8}, {-44, 8}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-6, Interval = 0.2));
    end simple_tracking;
















































































  end Examples;
  
  package blocks "blocks"
  model noise_ung "noise_sampled TODO: noch eine normal-distribution erzuegen (Box-Muller oder ziggurat)"
    extends Modelica.Blocks.Interfaces.SO;
    parameter Real amplitude = 1.0;
    parameter Real seed = 0.0;
    constant Real m = 111.11 "int(2 ^ 31 - 1)";

    function rand
      input Real t;
      input Real seed;
      output Real r;
    protected
      Real x;
      constant Integer m = 2147483647 "int(2 ^ 31 - 1)";
      constant Integer a = 16807 "7 ^ 5";
    algorithm
      x := t * seed+t+500;
      for i in 1:3 loop
        x := mod(x*x*a,m);
      end for;
      r := x / m;
    end rand;
  equation
    y = (rand(time * m, seed) - 0.5) * amplitude;
    annotation(x(flags = 2), y(flags = 2), Icon(coordinateSystem(extent = {{-101.7, -51.7}, {101.7, 51.7}}), graphics = {Rectangle(lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-80, 56.7}, {53.3, -40}}), Line(points = {{-70, -20}, {-66.7, 20}, {-66.7, -3.3}, {-63.3, 3.3}, {-63.3, 0}, {-63.3, 10}, {-60, -3.3}, {-56.7, 16.7}, {-56.7, -13.3}, {-53.3, 36.7}, {-50, -3.3}, {-50, 10}, {-46.7, -10}, {-43.3, 6.7}, {-43.3, -3.3}, {-36.7, -23.3}, {-36.7, -10}, {-33.3, -3.3}, {-30, -10}, {-30, -3.3}, {-30, 30}, {-26.7, 30}, {-20, 6.7}, {-20, -13.3}, {-16.7, -16.7}, {-16.7, 3.3}, {-13.3, -10}, {-13.3, 3.3}, {-13.3, -16.7}, {-13.3, 43.3}, {-10, -16.7}, {-6.7, -6.7}, {-6.7, -13.3}, {-3.3, 16.7}, {-3.3, -3.3}, {-3.3, 3.3}, {0, -10}, {3.3, -13.3}, {6.7, 23.3}, {10, 6.7}, {13.3, 0}, {16.7, -6.7}, {16.7, -16.7}, {16.7, -3.3}, {20, -23.3}, {23.3, -6.7}, {23.3, -3.3}, {33.3, -23.3}, {33.3, 6.7}, {36.7, 6.7}, {40, 10}, {43.3, 46.7}, {40, -10}, {40, -13.3}, {43.3, 0}, {46.7, 10}, {50, -6.7}, {50, 3.3}, {53.3, 13.3}}, color = {0, 0, 0})}), experiment(StopTime = 1, StartTime = 0), Documentation(info = "<html>
for further information look here:

<a href=https://en.wikipedia.org/wiki/Linear_congruential_generator>Linear congruential generator</a>

https://en.wikipedia.org/wiki/List_of_random_number_generators
</html>"));
  end noise_ung;




  model noise_sampled
    parameter Real seed=99;
    parameter Real variance = 0.1;
    parameter Real samplePeriod = 0.1;
    Modelica.Blocks.Math.Add add1 annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    blocks.noise_ung noise_ung1(amplitude = variance, seed = seed)  annotation(Placement(visible = true, transformation(origin = {-40, 20}, extent = {{-10.17, -5.17}, {10.17, 5.17}}, rotation = 0)));
    Modelica.Blocks.Discrete.ZeroOrderHold zeroorderhold1(samplePeriod = samplePeriod) annotation(Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-92, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(u, add1.u2) annotation(Line(points = {{-102, 0}, {-39, 0}, {-39, -6}, {8, -6}}, color = {0, 0, 127}));
    connect(zeroorderhold1.y, y) annotation(Line(points = {{71, 0}, {92.5852, 0}, {92.5852, 0.801603}, {92.5852, 0.801603}}));
    connect(add1.y, zeroorderhold1.u) annotation(Line(points = {{31, 0}, {46.493, 0}, {46.493, 0}, {46.493, 0}}));
    connect(noise_ung1.y, add1.u1) annotation(Line(points = {{-29, 20}, {-9.218439999999999, 20}, {-9.218439999999999, 6.81363}, {8, 6.81363}, {8, 6}}));
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
  end noise_sampled;



  model noisetest
    noise_ung noise_ung1(seed = 1.0) annotation(Placement(visible = true, transformation(extent = {{-9, -1}, {12, 10}}, rotation = 0)));
    annotation(noise_ung1(y(flags = 2)), experiment(StopTime = 1, StartTime = 0, Tolerance = 1e-6, Interval = 0.002));
  end noisetest;

  model sampled_noise_noevent
    noise_ung noise_ung1 annotation(Placement(visible = true, transformation(extent = {{-43, -5}, {-22, 6}}, rotation = 0)));
    blocks.no_event_sampler no_event_sampler1 annotation(Placement(visible = true, transformation(origin = {34, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    function sampler
      input Real current_value;
      input Real last_value;
      output Real sampled_value;
    protected
      Real x;
      constant Integer m = 2147483647 "int(2 ^ 31 - 1)";
      constant Integer a = 16807 "7 ^ 5";
    algorithm
      sampled_value := current_value;
    end sampler;
  equation
    connect(noise_ung1.y, no_event_sampler1.u) annotation(Line(points = {{-21, 0.5}, {22, 0.5}, {22, 0}}, color = {0, 0, 127}));
    annotation(noise_ung1(y(flags = 2)), experiment(StopTime = 1, StartTime = 0, Tolerance = 1e-6, Interval = 0.002));
  end sampled_noise_noevent;


  model no_event_sampler
    extends Modelica.Blocks.Interfaces.DiscreteSISO;
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end no_event_sampler;

  model sample_test
    Modelica.Blocks.Discrete.ZeroOrderHold zeroorderhold1(samplePeriod = 0.1) annotation(Placement(visible = true, transformation(origin = {30, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Discrete.Sampler sampler1(samplePeriod = 0.1) annotation(Placement(visible = true, transformation(origin = {30, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Sine sine1(freqHz = 2) annotation(Placement(visible = true, transformation(origin = {-62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(sine1.y, zeroorderhold1.u) annotation(Line(points = {{-51, 0}, {-22, 0}, {-22, -12}, {16, -12}, {16, -12}}, color = {0, 0, 127}));
    connect(sine1.y, sampler1.u) annotation(Line(points = {{-51, 0}, {-22, 0}, {-22, 32}, {18, 32}, {18, 32}}, color = {0, 0, 127}));
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
  end sample_test;

  block OnOffIdleController "On-off controller"
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput reference "Connector of Real input signal used as reference signal" annotation(Placement(transformation(extent = {{-140, 80}, {-100, 40}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u "Connector of Real input signal used as measurement signal" annotation(Placement(transformation(extent = {{-140, -40}, {-100, -80}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal used as actuator signal" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
    parameter Real bandwidth(start = 0.1) "Bandwidth around reference signal";
  equation
    y = if u > reference + bandwidth / 2 then 1 else if u < reference - bandwidth / 2 then -1 else 0;
    annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1), graphics = {Text(extent = {{-92, 74}, {44, 44}}, lineThickness = 0.5, textString = "reference"), Text(extent = {{-94, -52}, {-34, -74}}, textString = "u"), Line(points = {{-76.0, -32.0}, {-68.0, -6.0}, {-50.0, 26.0}, {-24.0, 40.0}, {-2.0, 42.0}, {16.0, 36.0}, {32.0, 28.0}, {48.0, 12.0}, {58.0, -6.0}, {68.0, -28.0}}, color = {0, 0, 127}), Line(points = {{-78.0, -2.0}, {-6.0, 18.0}, {82.0, -12.0}}, color = {255, 0, 0}), Line(points = {{-78.0, 12.0}, {-6.0, 30.0}, {82.0, 0.0}}), Line(points = {{-78.0, -16.0}, {-6.0, 4.0}, {82.0, -26.0}}), Line(points = {{-82.0, -18.0}, {-56.0, -18.0}, {-56.0, -40.0}, {64.0, -40.0}, {64.0, -20.0}, {90.0, -20.0}}, color = {255, 0, 255})}), Documentation(info = "<html>
<p>The block OnOffController sets the output signal <b>y</b> to <b>true</b> when
the input signal <b>u</b> falls below the <b>reference</b> signal minus half of
the bandwidth and sets the output signal <b>y</b> to <b>false</b> when the input
signal <b>u</b> exceeds the <b>reference</b> signal plus half of the bandwidth.</p>
</html>"));
  end OnOffIdleController;

  package examples
    model onoffidletest
      Modelica.Blocks.Sources.Sine sine1 annotation(Placement(visible = true, transformation(origin = {-72, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OnOffIdleController onoffidlecontroller1(bandwidth = 0.05) annotation(Placement(visible = true, transformation(origin = {-18, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant const(k = 0.2) annotation(Placement(visible = true, transformation(origin = {-72, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(const.y, onoffidlecontroller1.reference) annotation(Line(points = {{-61, 42}, {-30, 42}, {-30, 42}, {-30, 42}}, color = {0, 0, 127}));
      connect(sine1.y, onoffidlecontroller1.u) annotation(Line(points = {{-61, 10}, {-50, 10}, {-50, 30}, {-30, 30}}, color = {0, 0, 127}));
      annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
    end onoffidletest;
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end examples;

  model timed_switch
    extends Modelica.Blocks.Interfaces.SISO;
    parameter Real switchTime = 0 "time where signal is turned on";
    Modelica.Blocks.Math.Product product1 annotation(Placement(visible = true, transformation(origin = {6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(startTime = switchTime) annotation(Placement(visible = true, transformation(origin = {-46, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(step1.y, product1.u1) annotation(Line(points = {{-34, 26}, {-22, 26}, {-22, 6}, {-6, 6}, {-6, 6}}, color = {0, 0, 127}));
    connect(product1.y, y) annotation(Line(points = {{18, 0}, {104, 0}, {104, 0}, {110, 0}}, color = {0, 0, 127}));
    connect(u, product1.u2) annotation(Line(points = {{-120, 0}, {-60, 0}, {-60, -6}, {-6, -6}, {-6, -6}}, color = {0, 0, 127}));
    annotation(Icon(graphics = {Text(origin = {-1, -13}, extent = {{-73, 17}, {73, -17}}, textString = "%switchTime"), Text(origin = {0, 55}, extent = {{-64, 23}, {64, -23}}, textString = "time")}), Diagram);
  end timed_switch;

    model gauss_noise "noise_sampled TODO: noch eine normal-distribution erzuegen (Box-Muller oder ziggurat)"
      extends Modelica.Blocks.Interfaces.SO;
      parameter Real amplitude = 1.0;
      parameter Real seed = 0.0;
      constant Real m = 111.11 "int(2 ^ 31 - 1)";

      function rand
        input Real t;
        input Real seed;
        output Real r;
      protected
        Integer x;
        constant Integer m = 2147483647 "int(2 ^ 31 - 1)";
        constant Integer a = 16807 "7 ^ 5";
      algorithm
        x := t * seed + t + 500;
        for i in 1:3 loop
          //use Xorshift here instead?
          x := mod(x * x, m);
        end for;
        r := x/m;
      end rand;
    equation
      y = (rand(time * m, seed) - 0.5) * amplitude;
      annotation(
        x(flags = 2),
        y(flags = 2),
        Icon(coordinateSystem(extent = {{-101.7, -51.7}, {101.7, 51.7}}), graphics = {Rectangle(lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-80, 56.7}, {53.3, -40}}), Line(points = {{-70, -20}, {-66.7, 20}, {-66.7, -3.3}, {-63.3, 3.3}, {-63.3, 0}, {-63.3, 10}, {-60, -3.3}, {-56.7, 16.7}, {-56.7, -13.3}, {-53.3, 36.7}, {-50, -3.3}, {-50, 10}, {-46.7, -10}, {-43.3, 6.7}, {-43.3, -3.3}, {-36.7, -23.3}, {-36.7, -10}, {-33.3, -3.3}, {-30, -10}, {-30, -3.3}, {-30, 30}, {-26.7, 30}, {-20, 6.7}, {-20, -13.3}, {-16.7, -16.7}, {-16.7, 3.3}, {-13.3, -10}, {-13.3, 3.3}, {-13.3, -16.7}, {-13.3, 43.3}, {-10, -16.7}, {-6.7, -6.7}, {-6.7, -13.3}, {-3.3, 16.7}, {-3.3, -3.3}, {-3.3, 3.3}, {0, -10}, {3.3, -13.3}, {6.7, 23.3}, {10, 6.7}, {13.3, 0}, {16.7, -6.7}, {16.7, -16.7}, {16.7, -3.3}, {20, -23.3}, {23.3, -6.7}, {23.3, -3.3}, {33.3, -23.3}, {33.3, 6.7}, {36.7, 6.7}, {40, 10}, {43.3, 46.7}, {40, -10}, {40, -13.3}, {43.3, 0}, {46.7, 10}, {50, -6.7}, {50, 3.3}, {53.3, 13.3}}, color = {0, 0, 0})}),
        experiment(StopTime = 1, StartTime = 0),
        Documentation(info = "<html>
for further information look here:

<a href=https://en.wikipedia.org/wiki/Linear_congruential_generator>Linear congruential generator</a>

https://en.wikipedia.org/wiki/List_of_random_number_generators
</html>"));
    end gauss_noise;

model kalman "Kalman Filter for Modelica"
  import Modelica.Math.Matrices.*;
  parameter Real dT=1.0 "sample period";
  Modelica.Blocks.Interfaces.RealVectorInput u[nu] "input of control vector" annotation(
    Placement(visible = true, transformation(origin = {-92, 68}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-84, 58}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealVectorInput z[nz] "input of measurements)" annotation(
    Placement(visible = true, transformation(origin = {-91, -1}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-84, 0}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput x[nx] "state" annotation(
    Placement(visible = true, transformation(origin = {94, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {94, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real A[:, size(A, 1)]=[1, dT; 0, 1]
    "Matrix A of state space model (e.g., A=[1, 0; 0, 1])";
  parameter Real B[size(A, 1), :]=[dT*dT*0.5; dT]
    "Matrix B of state space model (e.g., B=[1; 1])";
  parameter Real sigma_u = 0.1;
  parameter Real Q[nx,nx]=B*transpose(B)*sigma_u*sigma_u "process covariance"; 
  Real P[nx,nx](start=identity(nx)) "State Covariance";
  parameter Real H[:,size(H,1)]=[1,0;0,1] "measurement function";
  Real xp[nx] "prior state";
  Real y[nz] "residual";
  Real K[nx,nz] "Kalman Gain";
  parameter Real R[nz,nz]=[0.5,0;0,0.5] "Noise Covariance (corresponding to z)";
  Real S[nz,nz] "system uncertainty ";
protected
  parameter Integer nz = size(H, 1) "number of measurements";
  parameter Integer nu = size(B, 2);
  parameter Integer nx = size(A, 1) "number of states";
  parameter Real startTime = 1.0;
  protected output Boolean sampleTrigger "True, if sample time instant";
  protected output Boolean firstTrigger "Rising edge signals first sample";
equation 
  sampleTrigger = sample(startTime, dT);
 algorithm
  when sampleTrigger then
  firstTrigger := time <= startTime + 0.5 * dT;
  //prediction
     xp := A*pre(x) + B*u;
     //y := C*pre(x) + D*u;
     P := A*pre(P)*transpose(A) + Q "State Covariance prediction";
  //measurement update
     y := z - H * xp;// residual
     S := H*P*transpose(H)+R;
     K := P*transpose(H)*inv(S);
     x := xp + K*y; //state update
     P := (identity(nx) - K*H)*P;  //update of covariance
    end when;
  annotation(
    Icon(graphics = {Text(origin = {27, -31}, lineColor = {144, 149, 7}, fillColor = {222, 222, 222}, extent = {{-83, 95}, {55, -33}}, textString = "K", textStyle = {TextStyle.Bold}), Rectangle(origin = {0, 2}, extent = {{-84, 82}, {84, -82}}), Text(origin = {-48, 13}, extent = {{-20, 9}, {20, -31}}, textString = "z"), Text(origin = {-48, 71}, extent = {{-20, 9}, {20, -31}}, textString = "u")}, coordinateSystem(initialScale = 0.1)),
        Documentation(info = "<html><head></head><body>
<pre style=\"margin-top: 0px; margin-bottom: 0px;\"><span style=\" font-family:'DejaVu Sans Mono'; font-size:12pt; color:#008b00;\">implementation according to:</span></pre><pre style=\"margin-top: 0px; margin-bottom: 0px;\"><span style=\" font-family:'DejaVu Sans Mono'; font-size:12pt; color:#008b00;\"><br></span></pre><pre style=\"margin-top: 0px; margin-bottom: 0px;\"><!--StartFragment--><span style=\" font-family:'DejaVu Sans Mono'; font-size:12pt; color:#008b00;\">https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/06-Multivariate-Kalman-Filters.ipynb</span><!--EndFragment--></pre><pre style=\"margin-top: 0px; margin-bottom: 0px;\"><span style=\" font-family:'DejaVu Sans Mono'; font-size:12pt; color:#008b00;\"><br></span></pre><pre style=\"margin-top: 0px; margin-bottom: 0px;\">

<pre style=\"margin-top: 0px; margin-bottom: 0px;\"><!--StartFragment--><span style=\"font-family: 'DejaVu Sans Mono'; font-size: 12pt;\">  </span><span style=\" font-family:'DejaVu Sans Mono'; font-size:12pt; color:#009600;\">//https://docs.scipy.org/doc/scipy-0.15.1/reference/generated/scipy.signal.cont2discrete.html</span><!--EndFragment--></pre></pre></body></html>"));
end kalman;




























































































































    model kalman_continuos
      //https://docs.scipy.org/doc/scipy-0.15.1/reference/generated/scipy.signal.cont2discrete.html
      //parameter Integer nz = 1 "number of states";
      Modelica.Blocks.Interfaces.RealVectorInput u[nu] "input of control vector" annotation(
        Placement(visible = true, transformation(origin = {-92, 68}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-84, 58}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealVectorInput z[nz] "input of measurements)" annotation(
        Placement(visible = true, transformation(origin = {-91, -1}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-84, 0}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y[ny] annotation(
        Placement(visible = true, transformation(origin = {94, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {94, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Real A[:, size(A, 1)]=[0, 1; 0, 0]
        "Matrix A of state space model (e.g., A=[1, 0; 0, 1])";
      parameter Real B[size(A, 1), :]=[0; 1]
        "Matrix B of state space model (e.g., B=[1; 1])";
      parameter Real C[:, size(A, 1)]=[1, 0]
        "Matrix C of state space model (e.g., C=[1, 1])";
      parameter Real D[size(C, 1), size(B, 2)]=zeros(size(C, 1), size(B, 2))
        "Matrix D of state space model";
  Modelica.Blocks.Continuous.StateSpace stateSpace1(A=A, B=B, C=C, D=D) annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      parameter Integer nz = 3 "number of states";
      parameter Integer nu = size(B, 2);
      parameter Integer nx = size(A, 1) "number of states";
      parameter Integer ny = size(C, 1) "number of outputs";
    equation
      connect(stateSpace1.y, y) annotation(
        Line(points = {{12, 24}, {44, 24}, {44, 2}, {94, 2}, {94, 0}}, color = {0, 0, 127}));
      connect(u, stateSpace1.u) annotation(
        Line(points = {{-92, 68}, {-64, 68}, {-64, 24}, {-12, 24}, {-12, 24}}, color = {0, 0, 127}));
//prediction
    
      annotation(
        Icon(graphics = {Text(origin = {27, -31}, lineColor = {144, 149, 7}, fillColor = {222, 222, 222}, extent = {{-83, 95}, {55, -33}}, textString = "K", textStyle = {TextStyle.Bold}), Rectangle(origin = {0, 2}, extent = {{-84, 82}, {84, -82}}), Text(origin = {-48, 13}, extent = {{-20, 9}, {20, -31}}, textString = "z"), Text(origin = {-48, 71}, extent = {{-20, 9}, {20, -31}}, textString = "u")}, coordinateSystem(initialScale = 0.1)));
    end kalman_continuos;





































































  annotation(dateModified = "2014-04-17 11:12:16Z");
end blocks;


  annotation(
    uses(Modelica(version = "3.2.1")));
end kalman;