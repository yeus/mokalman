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
  model mass_estimate
      inner Modelica.Mechanics.MultiBody.World world annotation(
        Placement(visible = true, transformation(origin = {-64, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    end mass_estimate;


  end Examples;

  annotation(
    uses(Modelica(version = "3.2.1")));
end kalman;