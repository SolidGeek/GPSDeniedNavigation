#include "visual_odemetry.h"


VisualOdemetry::VisualOdemetry(  )
{
    // Nothing
}

void VisualOdemetry::config( int frame_width, int frame_height, int interval, float scale ){

    // Calculate scaled size
    int scale_width = (float)frame_width * scale;
    int scale_height = (float)frame_height * scale;
    this->size = cv::Size(scale_width, scale_height);

    // Set interval for points to track
    this->interval = interval;
    int features_count = (scale_width * scale_height)/pow(interval, 2);

    // Initialize feature tracker
    this->tracker = new FeatureTracker( this->status, features_count );

    printf("Scaled frame size is: %d x %d px (width x height) \n", scale_width, scale_height);
    printf("Tracking %d No. of points for sparse method \n", features_count);
}

void VisualOdemetry::process_frame( cv::Mat raw, cv::Mat &output ){
    // Resize the frame to specified size
    cv::resize(raw, output, this->size);

    this->frame_gfx = output;

    // Convert to grayscale (and use this to perform flow analysis)
    // cv::cvtColor(this->frame, output, cv::COLOR_BGR2GRAY);
}

void VisualOdemetry::compute_dense_flow( cv::Mat raw, float dt )
{
    cv::Mat vectors;    // Calculated flow vectors

    float xsum, ysum = 0;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, this->frame );
    
    if( !this->frame_prev.empty() ){

        // Use opencv to calculate flow between new frame and previous frame
        cv::calcOpticalFlowFarneback( this->frame_prev, this->frame, vectors, this->pyr_scale, this->levels, this->winsize, this->iterations, this->poly_n, this->poly_sigma, 0);

        for (int y = this->interval/2; y < vectors.rows; y+=this->interval)
        {
            for (int x = this->interval/2; x < vectors.cols; x+=this->interval)
            {
                // Extract the vector the flow in this 
                cv::Point2f vector = vectors.at<cv::Point2f>(y,x);

                xsum += vector.x;
                ysum += vector.y;

                // Visualize the flow in the frame
                cv::line(this->frame_gfx, cv::Point(x,y), cv::Point(x-int(vector.x),y-int(vector.y)), cv::Scalar(255, 255, 255) );
                cv::circle(this->frame_gfx, cv::Point(x,y), 2, cv::Scalar(255, 255, 255), -1 );

                count++;
            }
        }

        if( dt != 0 ){
            // Calculate flow from sums
            this->of_x = this->compute_pixel_velocity( xsum, count, dt);
            this->of_y = this->compute_pixel_velocity( ysum, count, dt);
        }
    }

    this->frame_prev = this->frame;
}

void VisualOdemetry::compute_sparse_flow( cv::Mat raw, float dt ){

    float xsum, ysum = 0;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, this->frame );

    // Track features in new frame
    tracker->track_features(this->frame, this->features_current, this->status );

    if (!this->features_current.empty() && !this->features_previous.empty()) {

        // Calculate pixel flow

		for (int i = 0; i < this->status.size(); i++) {
			//just use active features
			if ( status[i] == 1 ) {
				xsum += features_current[i].x - features_previous[i].x;
				ysum += features_current[i].y - features_previous[i].y;

				count++;
			}
		}

        if( count ){
            // Calculate pixel flow means
            float pixel_mean_x = xsum / count;
            float pixel_mean_y = ysum / count;

            float pixel_variance_x = 0;
            float pixel_variance_y = 0;

            // Calculate sample variance (sum)
			for (int i = 0; i < status.size(); i++) {
				if (status[i] == 1) {
					pixel_variance_x += pow(features_current[i].x - features_previous[i].x - pixel_mean_x, 2);
					pixel_variance_y += pow(features_current[i].y - features_previous[i].y - pixel_mean_y, 2);
				}
			}

            // Convert to standard deviation
			pixel_variance_x = sqrt(pixel_variance_x / count);
			pixel_variance_y = sqrt(pixel_variance_y / count);

            // Now remove outliers based on standard deviation and a confidense interval of 90%
            float z_multiplier = 1.65; // 90% confidense interval 

            float xsum_confidense = 0;
            float ysum_confidense = 0;
            int count_confidense = 0;

            for (int i = 0; i < status.size(); i++) {
                if( status[i] == 1){
                    float val_x = features_current[i].x - features_previous[i].x;
					float val_y = features_current[i].y - features_previous[i].y;
					//check if inside confidence interval

                    int length = (int) ceil( sqrt( pow(val_x,2) + pow(val_y,2) ));

					if ( (fabs(val_x - pixel_mean_x) < pixel_variance_x * z_multiplier && fabs(val_y - pixel_mean_y) < pixel_variance_y * z_multiplier) || length <= 1 ) {
						xsum_confidense += val_x;
						ysum_confidense += val_y;

                        // Visualize the flow in the frame
                        cv::line(this->frame_gfx, cv::Point(features_previous[i].x, features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(255, 255, 255) );
                        cv::circle(this->frame_gfx, cv::Point(features_current[i].x, features_current[i].y), 2, cv::Scalar(255, 255, 255), -1 );

						count_confidense++; 
					} 
                }
            }
            // If some of the samples were inside the confidense interval
            if(count_confidense){
                xsum = xsum_confidense;
                ysum = ysum_confidense;
                count = count_confidense;
            }
            
            if( dt != 0 ){
                this->of_x = this->compute_pixel_velocity( xsum, count, dt);
                this->of_y = this->compute_pixel_velocity( ysum, count, dt);
            }
        }else{
            this->of_x = 0;
            this->of_y = 0;
        }
	}

    this->features_previous = this->features_current;

    this->tracker->update_feature_status( this->status );

    this->frame_prev = this->frame;
}

float VisualOdemetry::compute_pixel_velocity( float sum, int count, float dt ){
    // Calculate pixels per second
    return (sum / (float)count) / dt;
}

float VisualOdemetry::compute_velocity( float flow, int axis_length, float distance ){
    // Find focal length in pixels 
    int focal_length_px = (axis_length/2) / tan( 64.0 /2 );

    // Scaling factor
    float pixel_per_length = (float)distance / focal_length_px;

    return flow * pixel_per_length;
}

cv::Mat VisualOdemetry::get_frame( void ){
    return this->frame_gfx;
}

float VisualOdemetry::get_vel_x( float distance ){
    if( distance != 0 ){
        return compute_velocity( this->of_x, this->frame.cols, distance );
    }

    return of_x;
}

float VisualOdemetry::get_vel_y( float distance ){
    if( distance != 0 ){
        return compute_velocity( this->of_y, this->frame.rows, distance );
    }

    return of_y;
}



