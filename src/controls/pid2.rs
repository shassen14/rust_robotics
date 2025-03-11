// initial/experimental. not sure how I want to program it yet
use crate::utils::math;

// TODO: implement with generic type if it's simple enough
pub struct Controller {
    kp: Vec<f64>,
    ki: Vec<f64>,
    kd: Vec<f64>,
    error_previous: Vec<f64>,
    error_current: Vec<f64>,
    error_total: Vec<f64>,
    error_total_lower_bound: Vec<f64>,
    error_total_upper_bound: Vec<f64>,
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
            error_previous: vec![0.0; n],
            error_current: vec![0.0; n],
            error_total: vec![0.0; n],
            error_total_lower_bound: lower_bound,
            error_total_upper_bound: upper_bound,
        }
    }

    /// TODO: Figure out why you have this
    /// Maybe this is for automatic gain tuning????
    ///
    /// # Arguments
    ///
    ///
    pub fn update_gains(&mut self, kp: &Vec<f64>, ki: &Vec<f64>, kd: &Vec<f64>) {
        self.kp = kp.clone();
        self.ki = ki.clone();
        self.kd = kd.clone();
    }

    pub fn compute(&mut self, error: &Vec<f64>) -> Vec<f64> {
        assert_eq!(self.error_total.len(), error.len());

        let n = self.error_total.len();

        self.error_previous = self.error_current.clone();
        self.error_current = error.clone();

        for i in 0..n {
            self.error_total[i] += self.error_current[i];
        }

        self.error_total = math::bound_value(
            self.error_total.clone(),
            self.error_total_lower_bound.clone(),
            self.error_total_upper_bound.clone(),
        );

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
