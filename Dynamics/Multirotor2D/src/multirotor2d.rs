#[derive(Debug)]
pub struct Multirotor2DState {
    pub x: f32,
    pub x_dot: f32,
    pub z: f32,
    pub z_dot: f32,
    pub theta: f32,
    pub theta_dot: f32,
}

#[derive(Debug)]
pub struct Multirotor2DAction {
    pub f1: f32,
    pub f2: f32,
}

#[derive(Debug)]
pub struct Multirotor2D {
    pub mass: f32,
    pub g: f32,
    pub jyy: f32,
    pub l: f32,
    pub dt: f32,
    pub state: Multirotor2DState,
}


impl Multirotor2D {
    pub fn step(&mut self, action: Multirotor2DAction) {
            let pi_f32 = std::f32::consts::PI;

            let x_new: f32 = self.state.x + self.state.x_dot * self.dt;
            let x_dot_new: f32 = self.state.x_dot + ((-(action.f1 + action.f2)*self.state.theta.sin()) / self.mass)*self.dt;
            let z_new: f32 = self.state.z + self.state.z_dot * self.dt;
            let z_dot_new: f32 = self.state.z_dot + (((action.f1 + action.f2)*self.state.theta.cos()) / self.mass - self.g)*self.dt;
            let mut theta_new: f32 = self.state.theta + self.state.theta_dot*self.dt;
            let theta_dot_new: f32 = self.state.theta_dot + ((action.f2 - action.f1)*self.l/self.jyy)*self.dt;

            theta_new = theta_new % (2.0*pi_f32);
            if theta_new >= pi_f32 {
                theta_new  = theta_new - 2.0*pi_f32;
            }
            else if theta_new <= -pi_f32 {
                theta_new  = theta_new + 2.0*pi_f32;
            }

            self.state.x = x_new;
            self.state.x_dot = x_dot_new;
            self.state.z = z_new;
            self.state.z_dot = z_dot_new;
            self.state.theta = theta_new;
            self.state.theta_dot = theta_dot_new;
    }
}