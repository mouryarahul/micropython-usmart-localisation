import math

try:
    import ulab as np
    from uac_localisation.main.misc.utils import random_combinations
    from uac_localisation.main.utm.conversion import to_latlon, from_latlon
    use_ulab = True
except ImportError:
    import numpy as np
    use_ulab = False
    from main.misc.utils import random_combinations
    from main.utm.conversion import to_latlon, from_latlon


class TDOALocalization:
    def __init__(self, soundspeed, depth):
        self._soundspeed = soundspeed
        self._depth = depth

    @property
    def sound_speed(self):
        return self._soundspeed

    @sound_speed.setter
    def sound_speed(self, soundspeed: float):
        """ Set the sound speed """
        self._soundspeed = soundspeed

    @property
    def sensor_depth(self):
        return self._depth

    @sensor_depth.setter
    def sensor_depth(self, depth: float):
        """ Set the sensor depth """
        self._depth = depth

    @staticmethod
    def extract_tdoa_info(soundspeed, beacon_signals: list) -> list:
        final_loc_range_diff_info = []
        while beacon_signals:
            segment = beacon_signals.pop(0)
            lead_anchor = segment.pop(0)
            x_lead, y_lead, zone_lead, zone_letter_lead = from_latlon(lead_anchor[0], lead_anchor[1])
            loc_range_diff_info = [[[zone_lead, zone_letter_lead], [x_lead, y_lead, lead_anchor[2]]]]
            while segment:
                asst_anchor = segment.pop(0)
                x_asst, y_asst, zone_asst, zone_letter_asst = from_latlon(asst_anchor[0], asst_anchor[1])
                if (zone_lead != zone_asst) or (zone_letter_lead != zone_letter_asst):
                    break
                anchor_range = math.sqrt((x_lead - x_asst) ** 2 + (y_lead - y_asst) ** 2 + (lead_anchor[2] - asst_anchor[2]) ** 2)
                range_diff = soundspeed * (asst_anchor[4] - (asst_anchor[3] - lead_anchor[3])) + anchor_range
                loc_range_diff_info.append([x_asst, y_asst, asst_anchor[2], range_diff])

            final_loc_range_diff_info.append(loc_range_diff_info)

        return final_loc_range_diff_info

    @staticmethod
    def perform_tdoa_multilateration_closed_form(anchor_locations, range_diffs, sensor_depth):
        """
        Calculates the unknown location given the anchor locations and TDoA info
        :param anchor_locations: an array of dims [n,3]
        :param sensor_depth: float
        :param range_diffs: an array of dims [n-1,1]
        :return: an array of dim [3,1]
        """
        soln = np.array([])
        if use_ulab:
            bflag_proceed = anchor_locations.size() > 0 and range_diffs.size() > 0
        else:
            bflag_proceed = anchor_locations.size > 0 and range_diffs.size > 0

        if bflag_proceed:
            if use_ulab:
                n = range_diffs.size()
            else:
                n = range_diffs.size

            mat_m = np.zeros((n, 2), dtype=np.float)
            for i in range(n):
                mat_m[i, :] = np.array([(anchor_locations[0, 0] - anchor_locations[i + 1, 0]),
                                        (anchor_locations[0, 1] - anchor_locations[i + 1, 1])]) * 2.0
            vec_c = range_diffs * 2.0
            vec_d = np.zeros(n, dtype=np.float)
            for i in range(n):
                vec_d[i] = np.linalg.norm(anchor_locations[i + 1]) ** 2 - np.linalg.norm(anchor_locations[0]) ** 2 \
                           - range_diffs[i] ** 2 + 2.0 * (anchor_locations[0, 2] - anchor_locations[i + 1, 2]) * sensor_depth

            # Invert the matrix and find the solution if feasible
            if use_ulab:
                mTm = np.linalg.dot(mat_m.transpose(), mat_m)
            else:
                mTm = np.dot(mat_m.transpose(), mat_m)

            # Invert the matrix mTm
            denom = mTm[0, 0] * mTm[1, 1] - mTm[1, 0] * mTm[0, 1]
            numer = np.array([[mTm[1, 1], -mTm[0, 1]], [-mTm[1, 0], mTm[0, 0]]])
            if denom >= 1E-9:
                mTm_inv = numer / denom
                if use_ulab:
                    mat_m_inv = np.linalg.dot(mTm_inv, mat_m.transpose())
                else:
                    mat_m_inv = np.matmul(mTm_inv, mat_m.transpose())

                # Solve the quadratic equations
                if use_ulab:
                    vec_a = -np.linalg.dot(mat_m_inv, vec_c)
                    vec_b = -np.linalg.dot(mat_m_inv, vec_d)
                    alpha = np.linalg.dot(vec_a, vec_a) - 1.0
                    beta = 2.0 * np.linalg.dot(vec_a, (vec_b - anchor_locations[0][0:2]))
                    gamma = np.linalg.dot((vec_b - anchor_locations[0][0:2]), (vec_b - anchor_locations[0][0:2])) + (sensor_depth - anchor_locations[0][2]) ** 2
                else:
                    vec_a = -np.dot(mat_m_inv, vec_c)
                    vec_b = -np.dot(mat_m_inv, vec_d)
                    alpha = np.dot(vec_a, vec_a) - 1.0
                    beta = 2.0 * np.dot(vec_a, (vec_b - anchor_locations[0][0:2]))
                    gamma = np.dot((vec_b - anchor_locations[0][0:2]), (vec_b - anchor_locations[0][0:2])) + (sensor_depth - anchor_locations[0][2]) ** 2

                delta = beta ** 2 - 4.0 * alpha * gamma

                if delta < 0.0:
                    pass
                elif math.isclose(delta, 0.0, abs_tol=1E-5) and beta < 0.0:
                    root = -beta / (2.0 * alpha)
                    soln = vec_a * root + vec_b
                    soln = np.array([soln[0], soln[1], sensor_depth])
                elif math.isclose(alpha, 0.0, abs_tol=1E-5) and beta < 0.0:
                    root = -gamma / beta
                    soln = vec_a * root + vec_b
                    soln = np.array([soln[0], soln[1], sensor_depth])
                elif delta > 0.0 and not math.isclose(alpha, 0.0, abs_tol=1E-5):
                    # print("delta > 0 and alpha != 0")
                    sqrt_delta = np.sqrt(delta)
                    root1 = (-beta - sqrt_delta) / (2 * alpha)
                    root2 = (-beta + sqrt_delta) / (2 * alpha)
                    # print("Root1={:0.4f} and Root2={:0.4f}".format(root1, root2))
                    if root2 < 0.0 < root1:
                        soln = vec_a * root1 + vec_b
                        soln = np.array([soln[0], soln[1], sensor_depth])
                    elif root1 < 0.0 < root2:
                        soln = vec_a * root2 + vec_b
                        soln = np.array([soln[0], soln[1], sensor_depth])
                    elif root1 > 0.0 and root2 > 0.0:
                        pass
                    else:  # Both roots are negative
                        pass
                else:
                    pass
            else:
                pass

        return soln

    @staticmethod
    def tdoa_multilateration_from_multiple_segments_gauss_newton(anchor_locations: list, range_diffs: list,
                                                                 init_soln: list, max_iters: int = 10,
                                                                 eta: float = 1E-3, abs_tol: float = 1E-6):
        # Gauss-Newton Iterations
        soln = np.array(init_soln)
        # Main iterations
        for j in range(max_iters):
            # ==== Calculate the LHS and RHS of Normal Equation: J'*J*d = -J'*res,
            # where J is Jacobian, d is descent direction, res is residual ====== #
            jTj = np.zeros((2, 2), dtype=np.float)
            jTres = np.zeros((2, 1), dtype=np.float)
            for locations, diffs in zip(anchor_locations, range_diffs):
                if use_ulab:
                    m = locations.shape()[0]
                else:
                    m = locations.shape[0]
                denom_1 = np.linalg.norm(soln - locations[0])
                for i in range(1, m):
                    denom_2 = np.linalg.norm(soln - locations[i])
                    res = (diffs[i - 1] - (denom_1 - denom_2))
                    del_res = np.array([[((soln[0] - locations[i][0]) / denom_2 - (soln[0] - locations[0][0]) / denom_1)],
                                        [((soln[1] - locations[i][1]) / denom_2 - (soln[1] - locations[0][1]) / denom_1)]])
                    if use_ulab:
                        jTj = jTj + np.linalg.dot(del_res, del_res.transpose())
                    else:
                        jTj = jTj + np.dot(del_res, del_res.transpose())

                    jTres = jTres + (del_res * res)
                # ===================== calculation ENDs here  ======================================================= #
            # Take next Step:
            # Modification according to Levenberg-Marquardt suggestion
            jTj = jTj + np.diag(np.diag(jTj) * eta)
            eta = eta / 2.0

            # Invert 2x2 matrix
            denom = jTj[0][0] * jTj[1][1] - jTj[1][0] * jTj[0][1]
            numer = np.array([[jTj[1][1], -jTj[0][1]], [-jTj[1][0], jTj[0][0]]])
            if denom >= 1E-9:
                inv_jTj = numer / denom
                if use_ulab:
                    temp = np.linalg.dot(inv_jTj, jTres)
                else:
                    temp = np.dot(inv_jTj, jTres)

                del_soln = np.array([temp[0][0], temp[1][0], 0.0])
                soln = soln - del_soln
                if np.linalg.norm(del_soln) <= abs_tol:
                    break
            else:
                print("Can't invert the matrix!")
                break

        return soln

    @staticmethod
    def tdoa_multilateration_from_all_segments_gauss_newton(loc_range_diff_info: list, depth: float,
                                                            max_iters: int = 10, eta: float = 1E-3, abs_tol: float = 1E-6):
        num_segment = len(loc_range_diff_info)
        anchor_location_list = []
        zone_list = []
        anchor_rangediff_list = []

        for i in range(num_segment):
            loc_range_diff = loc_range_diff_info[i]
            num_anchors = len(loc_range_diff)
            # create numpy array to hold anchor locations and range-differences
            anchor_locations = np.zeros((num_anchors, 3), dtype=np.float)
            range_diffs = np.zeros(num_anchors - 1, dtype=np.float)

            # extract locations and range-differences from list and store them into respective numpy array
            for j in range(num_anchors):
                if j == 0:
                    zone, zone_letter = loc_range_diff[j][0][0], loc_range_diff[j][0][1]
                    zone_list.append([zone, zone_letter])
                    anchor_locations[j] = np.array(loc_range_diff[j][1])
                else:
                    anchor_locations[j] = np.array(loc_range_diff[j][0:3])
                    range_diffs[j - 1] = loc_range_diff[j][3]

            # Collect anchor locations and range-differences arrays
            anchor_location_list.append(anchor_locations)
            anchor_rangediff_list.append(range_diffs)

        # Check that all zone and zone letters are same
        bflag_zone = True
        zone, zone_letter = zone_list[0][0], zone_list[0][1]
        for zone_info in zone_list:
            if zone_info != [zone, zone_letter]:
                bflag_zone = False
                break

        if not bflag_zone:
            raise ValueError("All the anchors are not in same zone!")

        # =========== Gauss Newton Iterations for Location Estimation  ========== #

        # Initial Solution is taken as Mean location of all the Anchors
        mean_anchor_location = np.zeros(3, dtype=np.float)
        num_anchors = 0
        for anchor_location in anchor_location_list:
            if use_ulab:
                mean_anchor_location = mean_anchor_location + np.sum(anchor_location, axis=0)
                num_anchors += anchor_location.shape()[0]
            else:
                mean_anchor_location = mean_anchor_location + np.sum(anchor_location, axis=0)
                num_anchors += anchor_location.shape[0]

        mean_anchor_location = mean_anchor_location / num_anchors
        soln = np.array(mean_anchor_location)
        soln[2] = depth  # replace 3rd coordinate with sensor depth
        # Main iterations
        for j in range(max_iters):
            # ==== Calculate the LHS and RHS of Normal Equation: J'*J*d = -J'*res,
            # where J is Jacobian, d is descent direction, res is residual ====== #
            jTj = np.zeros((2, 2))
            jTres = np.zeros((2, 1))
            for anchor_locations, range_diffs in zip(anchor_location_list, anchor_rangediff_list):
                if use_ulab:
                    m = anchor_locations.shape()[0]
                else:
                    m = anchor_locations.shape[0]
                denom_1 = np.linalg.norm(soln - anchor_locations[0])
                for i in range(1, m):
                    denom_2 = np.linalg.norm(soln - anchor_locations[i])
                    res = (range_diffs[i - 1] - (denom_1 - denom_2))
                    del_res = np.array([[((soln[0] - anchor_locations[i][0]) / denom_2 - (soln[0] - anchor_locations[0][0]) / denom_1)],
                                        [((soln[1] - anchor_locations[i][1]) / denom_2 - (soln[1] - anchor_locations[0][1]) / denom_1)]])
                    if use_ulab:
                        jTj = jTj + np.linalg.dot(del_res, del_res.transpose())
                    else:
                        jTj = jTj + np.dot(del_res, del_res.transpose())

                    jTres = jTres + (del_res * res)
                # ===================== calculation ENDs here  ======================================================= #
            # ============= Take the next step: ====================== #
            # Modification according to Levenberg-Marquardt suggestion
            jTj = jTj + np.diag(np.diag(jTj) * eta)
            eta = eta / 2.0

            # Invert 2x2 matrix
            denom = jTj[0][0] * jTj[1][1] - jTj[1][0] * jTj[0][1]
            numer = np.array([[jTj[1][1], -jTj[0][1]], [-jTj[1][0], jTj[0][0]]])
            if denom >= 1E-9:
                inv_jTj = numer / denom
                if use_ulab:
                    temp = np.linalg.dot(inv_jTj, jTres)
                else:
                    temp = np.dot(inv_jTj, jTres)

                del_soln = np.array([temp[0][0], temp[1][0], 0.0])
                soln = soln - del_soln
                if np.linalg.norm(del_soln) <= abs_tol:
                    break
            else:
                print("Can't invert the matrix!")
                break

        # Convert to Lat/Lon
        try:
            lat, lon = to_latlon(soln[0], soln[1], zone, zone_letter)
            estimated_location = [lat, lon, soln[2]]
        except Exception as e:
            print("Not a valid estimate: " + str(e))
            estimated_location = []

        return estimated_location

    @staticmethod
    def tdoa_multilateration_from_single_segement_gauss_newton(locations: np.array, deltas: np.array, initial_soln: np.array,
                                                               max_iters: int = 10, eta: float = 1E-3, abs_tol: float = 1E-6):
        """
        tdoa_multilateration_gauss_newton solves the problem: argmin_x  sum_{i=1}^{N} (d_i - || x - a_0 ||_2 - || x - a_i ||_2)^{2}
        by Gauss-Newton Method, which finds a local optimum.
        :param locations: Nx3 array of anchor locations
        :param deltas: (N-1)x1 array of range-differences
        :param initial_soln: 3x1 array of initial solution
        :param max_iters: int
        :param eta: float
        :param abs_tol: float
        :return: 3x1 array of estimated solution
        """
        if use_ulab:
            m, n = locations.shape()
        else:
            m, n = locations.shape

        soln = np.array(initial_soln)

        for j in range(max_iters):
            # ==== Calculate the LHS and RHS of Normal Equation: J'*J*d = -J'*res,
            # where J is Jacobian, d is descent direction, res is residual ====== #
            jTj = np.zeros((n-1, n-1))
            jTres = np.zeros((n-1, 1))
            denom_1 = np.linalg.norm(soln - locations[0])
            for i in range(1, m):
                denom_2 = np.linalg.norm(soln - locations[i])
                res = (deltas[i - 1] - (denom_1 - denom_2))
                del_res = np.array([[((soln[0] - locations[i, 0]) / denom_2 - (soln[0] - locations[0, 0]) / denom_1)],
                                    [((soln[1] - locations[i, 1]) / denom_2 - (soln[1] - locations[0, 1]) / denom_1)]])
                if use_ulab:
                    jTj = jTj + np.linalg.dot(del_res, del_res.transpose())
                else:
                    jTj = jTj + np.dot(del_res, del_res.transpose())

                jTres = jTres + (del_res * res)
            # ===================== calculation ENDs here  ======================================================= #

            # ==================== Take next Step ========================= #
            # Modification according to Levenberg-Marquardt suggestion
            jTj = jTj + np.diag(np.diag(jTj) * eta)
            eta = eta / 2.0

            # Invert 2x2 matrix and update solution
            denom = jTj[0, 0] * jTj[1, 1] - jTj[1, 0] * jTj[0, 1]
            numer = np.array([[jTj[1, 1], -jTj[0, 1]], [-jTj[1, 0], jTj[0, 0]]])
            if denom >= 1E-9:
                inv_jTj = numer / denom
                if use_ulab:
                    temp = np.linalg.dot(inv_jTj, jTres)
                else:
                    temp = np.dot(inv_jTj, jTres)

                del_soln = np.array([temp[0, 0], temp[1, 0], 0.0])
                soln = soln - del_soln
                if np.linalg.norm(del_soln) <= abs_tol:
                    break
            else:
                print("Can't invert the matrix!")
                break

        return soln

    def least_median_square_estimate(self, loc_range_diff_info: list, max_iters) -> list:
        num_segments = len(loc_range_diff_info)
        lead_anchor_locations_list = []
        asst_anchor_locations_list = []
        range_diffs_list = []
        zone_info_list = []
        lut = []

        for i in range(num_segments):
            loc_range_diff = loc_range_diff_info[i]
            num_anchors = len(loc_range_diff)
            lut += [i] * (num_anchors - 1)
            for j in range(num_anchors):
                if j == 0:
                    zone, zone_letter = loc_range_diff[j][0][0], loc_range_diff[j][0][1]
                    zone_info_list.append([zone, zone_letter])
                    lead_anchor_locations_list.append(loc_range_diff[j][1])
                else:
                    asst_anchor_locations_list.append(loc_range_diff[j][0:3])
                    range_diffs_list.append(loc_range_diff[j][3])

        # Check if all anchors belong to same zone
        bflag_zone = True
        zone, zone_letter = zone_info_list[0][0], zone_info_list[0][1]
        for zone_info in zone_info_list:
            if zone_info != [zone, zone_letter]:
                bflag_zone = False
                break

        if not bflag_zone:
            raise ValueError("All the anchors are not in same zone!")

        num_asst_anchors = len(lut)
        # Generate subsets of assistant anchors with each subset containing 3 elements
        subsets = list(random_combinations(range(num_asst_anchors), 3, max_iters))

        # For each subset, calculate the unknown location using multilateration
        # and then calculate median of square residues = (range_diff - (SA - SB))**2 ,
        # where A and B represent the lead and the assistant anchor, respectively, and
        # S represent estimated unknown location
        min_median = 10E+10
        robust_location = None
        iter_count = 0
        for subset in subsets:
            # Pack together the assistant and the corresponding lead anchor's location
            anchor_locations = []
            range_diffs = []
            for index in subset:
                lead_anchor = lead_anchor_locations_list[lut[index]]
                asst_anchor = asst_anchor_locations_list[index]
                anchors = np.array([lead_anchor, asst_anchor])
                range_diff = np.array([range_diffs_list[index]])
                anchor_locations.append(anchors)
                range_diffs.append(range_diff)

            # Calculate initial soln as mean of Anchor Positions
            mean_anchor_location = np.zeros(3, dtype=np.float)
            num_anchors = 0
            for anchor in anchor_locations:
                mean_anchor_location = mean_anchor_location + np.sum(anchor, axis=0)
                if use_ulab:
                    num_anchors += anchor.shape()[0]
                else:
                    num_anchors += anchor.shape[0]

            init_soln = mean_anchor_location / num_anchors
            init_soln[2] = self._depth

            # Estimate location from selected subset of anchors using Gauss-Newton method
            sensor_location = TDOALocalization.tdoa_multilateration_from_multiple_segments_gauss_newton(anchor_locations, range_diffs, init_soln)
            # For each estimated location calculate the square of residues, i.e., (observed_range_diff - calculated_range_diff)**2
            sq_residues = []
            for index in subset:
                lead_anchor = np.array(lead_anchor_locations_list[lut[index]])
                asst_anchor = np.array(asst_anchor_locations_list[index])
                range_diff = np.array([range_diffs_list[index]])
                lead_to_sensor = np.linalg.norm((lead_anchor - sensor_location))
                asst_to_sensor = np.linalg.norm((asst_anchor - sensor_location))
                estimated_range_diff = lead_to_sensor - asst_to_sensor
                sq_residues.append((range_diff - estimated_range_diff) ** 2)
            # Calculate Median of Square-Residues
            current_median = np.median(np.array(sq_residues))
            # Select the subset which results into lowest median of errors
            if current_median <= min_median:
                min_median = current_median
                robust_location = sensor_location

            if iter_count >= max_iters:
                break
            iter_count += 1

        # Convert the location into Lat/Lon system
        try:
            lat, lon = to_latlon(robust_location[0], robust_location[1], zone, zone_letter)
            estimated_location = [lat, lon, robust_location[2]]
        except Exception as e:
            print("Not a valid estimate: " + str(e))
            estimated_location = []

        return estimated_location

    @staticmethod
    def perform_robust_multilateration(loc_range_diff_info, depth):
        num_segment = len(loc_range_diff_info)
        estimated_locations = []
        # estimate location from each segment
        for i in range(num_segment):
            loc_range_diff = loc_range_diff_info[i]
            num_anchors = len(loc_range_diff)
            # create numpy array to hold anchor locations and range-differences
            locations = np.zeros((num_anchors, 3), dtype=np.float)
            range_diffs = np.zeros(num_anchors - 1, dtype=np.float)

            # extract locations and range-differences from list and store them into respective numpy array
            zone = None
            zone_letter = None
            for j in range(num_anchors):
                if j == 0:
                    zone, zone_letter = loc_range_diff[j][0][0], loc_range_diff[j][0][1]
                    locations[j] = np.array(loc_range_diff[j][1])
                else:
                    locations[j] = np.array(loc_range_diff[j][0:3])
                    range_diffs[j - 1] = loc_range_diff[j][3]

            # Call Gauss Newton Method for location estimation
            initial_soln = np.mean(locations, axis=0)
            # replace 3 coordinate with sensor depth
            initial_soln[2] = depth
            estimated_location = TDOALocalization.tdoa_multilateration_from_single_segement_gauss_newton(locations, range_diffs, initial_soln)
            # estimated_location = TDOALocalization.perform_tdoa_multilateration_closed_form(locations, range_diffs, depth)
            if use_ulab:
                bflag_estimated = estimated_location.size() > 0
            else:
                bflag_estimated = estimated_location.size > 0
            # Convert to Lat/Lon
            if bflag_estimated > 0:
                lat, lon = to_latlon(estimated_location[0], estimated_location[1], zone, zone_letter)
                estimated_locations.append([lat, lon, estimated_location[2]])

        return estimated_locations

    def estimate_location(self, beacon_signals: list) -> list:
        loc_range_diff_info = self.extract_tdoa_info(self._soundspeed, beacon_signals)

        if loc_range_diff_info:
            # estimated_locations = self.perform_robust_multilateration(loc_range_diff_info, self.depth)
            # estimated_location = self.tdoa_multilateration_from_all_segments_gauss_newton(loc_range_diff_info, self.depth)
            robust_location = self.least_median_square_estimate(loc_range_diff_info, 120)

            """
            for i in range(len(estimated_locations)):
                print("Estimated Location from {}th segment = {}: ".format(i, estimated_locations[i]))
            print("\nJointly estimated Location = \t {}".format(estimated_location))
            print("\nRobust estimated location = \t {}".format(robust_location))
            """
            return robust_location
        else:
            print("No sufficient info to estimate location!")
            return []
