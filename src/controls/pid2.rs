// initial/experimental. not sure how I want to program it yet
use crate::utils::math;
use serde::Deserialize;

#[derive(Deserialize)]
pub struct PIDConfig {
    pub pid_params: PIDParams,
}

#[derive(Deserialize)]
pub struct PIDParams {
    pub kp: Vec<f64>,
    pub ki: Vec<f64>,
    pub kd: Vec<f64>,
    pub error_total_lower_bound: Vec<f64>,
    pub error_total_upper_bound: Vec<f64>,
}

// TODO: implement with generic type if it's simple enough
// TODO: make the last 3 errors optional so they don't have to be included in the config
#[derive(Deserialize)]
pub struct Controller {
    kp: Vec<f64>,
    ki: Vec<f64>,
    kd: Vec<f64>,
    error_total_lower_bound: Vec<f64>,
    error_total_upper_bound: Vec<f64>,
    error_previous: Vec<f64>,
    error_current: Vec<f64>,
    error_total: Vec<f64>,
}

impl Controller {
    pub fn new(
        kp: Vec<f64>,
        ki: Vec<f64>,
        kd: Vec<f64>,
        lower_bound: Vec<f64>,
        upper_bound: Vec<f64>,
    ) -> Self {
        let n: usize = kp.len();
        assert_eq!(n, ki.len());
        assert_eq!(n, kd.len());
        assert_eq!(n, lower_bound.len());
        assert_eq!(n, upper_bound.len());

        Controller {
            kp: kp,
            ki: ki,
            kd: kd,
            error_total_lower_bound: lower_bound,
            error_total_upper_bound: upper_bound,
            error_previous: vec![0.0; n],
            error_current: vec![0.0; n],
            error_total: vec![0.0; n],
        }
    }

    /// TODO: Figure out why you have this
    /// Maybe this is for automatic gain tuning????
    ///
    /// # Arguments
    ///
    ///
    pub fn update_gains(&mut self, kp: &[f64], ki: &[f64], kd: &[f64]) {
        self.kp.clear();
        self.ki.clear();
        self.kd.clear();

        self.kp.extend_from_slice(kp);
        self.ki.extend_from_slice(ki);
        self.kd.extend_from_slice(kd);
    }

    /// TODO: Lots of cloning. Can I avoud this please?
    ///
    /// # Arguments
    ///
    ///
    pub fn compute(&mut self, error: &[f64]) -> Vec<f64> {
        assert_eq!(self.error_total.len(), error.len());

        let n = self.error_total.len();

        // Directly update error_previous and error_current without cloning.
        self.error_previous.copy_from_slice(&self.error_current);
        self.error_current.copy_from_slice(error);

        // Update error_total directly (without cloning).
        for i in 0..n {
            self.error_total[i] += self.error_current[i];
            self.error_total[i] = math::bound_value(
                self.error_total[i],
                self.error_total_lower_bound[i],
                self.error_total_upper_bound[i],
            );
        }

        //
        let mut p = vec![0.; n];
        let mut i = vec![0.; n];
        let mut d = vec![0.; n];
        let mut u = vec![0.; n];

        for j in 0..self.error_total.len() {
            p[j] = self.kp[j] * self.error_current[j];
            i[j] = self.ki[j] * self.error_total[j];
            d[j] = self.kd[j] * (self.error_current[j] - self.error_current[j]);
            u[j] = p[j] + i[j] + d[j];
        }

        u
    }
}
