#include "opticalflow.h"


OpticalFlow::OpticalFlow( int frame_width, int frame_height, int scaledown, int interval, float fov )
{
    int scale_width = frame_width/scaledown;
    int scale_height = frame_height/scaledown;

    // Calculate scaled size
    this->size = cv::Size(scale_width, scale_height);
    this->interval = interval;
    this->fov = fov * M_PI / 180.0; // Convert fov angle to radians 

    int features_count = (scale_width * scale_height)/pow(interval, 2);

    printf("Scaled frame size is: %d x %d px (width x height) \n", scale_width, scale_height);
    printf("Tracking %d No. of points for sparse method \n", features_count);

    // Initialize feature tracker
    this->tracker = new FeatureTracker( this->status, features_count );
}

void OpticalFlow::process_frame( cv::Mat raw, cv::Mat &output ){
    // Resize the frame to specified size
    cv::resize(raw, this->frame, this->size);

    // Convert to grayscale (and use this to perform flow analysis)
    cv::cvtColor(this->frame, output, cv::COLOR_BGR2GRAY);
}

cv::Point2f OpticalFlow::compute_dense_flow( cv::Mat raw, float dt, float distance )
{
    cv::Mat vectors;    // Calculated flow vectors
    cv::Mat gray;       // Output image for processing

    float xsum, ysum = 0;
    float xflow, yflow;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, gray );
    
    if( !this->prev_gray.empty() ){

        // Use opencv to calculate flow between new frame and previous frame
        cv::calcOpticalFlowFarneback( this->prev_gray, gray, vectors, this->pyr_scale, this->levels, this->winsize, this->iterations, this->poly_n, this->poly_sigma, 0);

        for (int y = this->interval/2; y < vectors.rows; y+=this->interval)
        {
            for (int x = this->interval/2; x < vectors.cols; x+=this->interval)
            {
                // Extract the vector the flow in this 
                cv::Point2f vector = vectors.at<cv::Point2f>(y,x);

                xsum += vector.x;
                ysum += vector.y;

                // Visualize the flow in the frame
                cv::line(this->frame, cv::Point(x,y), cv::Point(x-int(vector.x),y-int(vector.y)), cv::Scalar(255, 255, 255) );
                cv::circle(this->frame, cv::Point(x,y), 2, cv::Scalar(255, 255, 255), -1 );

                count++;
            }
        }

        // Calculate flow from sums
        this->flow.x = this->compute_pixel_velocity( xsum, count, dt);
        this->flow.y = this->compute_pixel_velocity( ysum, count, dt);
    }

    this->prev_gray = gray;

    if( distance != 0 ){

        this->velocity.x = this->compute_velocity( this->flow.x, gray.cols, distance );
        this->velocity.y = this->compute_velocity( this->flow.y, gray.rows, distance );

        return this->velocity;
    }

    return this->flow;
}

/* 
cv::Point2f OpticalFlow::compute_sparse_flow( cv::Mat raw, float dt, float distance ){

    cv::Mat vectors;    // Calculated flow vectors
    cv::Mat gray;       // Output image for processing
    std::vector<uchar> status;
    std::vector<float> errors;

    float xsum, ysum = 0;
    float xflow, yflow;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, gray );

    if( !this->prev_gray.empty() ){

        cv::calcOpticalFlowPyrLK( this->prev_gray, gray, this->grid, vectors, status, errors, cv::Size(21, 21), 3 );
        this->grid = vectors;

        for (int y = 0; y < vectors.rows; y++)
        {
            for (int x = 0; x < vectors.cols; x++)
            {
                cv::Point2f start   = grid.at<cv::Point2f>(y,x);
                cv::Point2f end     = vectors.at<cv::Point2f>(y,x);
       
                if( status[count] == 1 ){
                    
                    if( errors[count] < 2.5f )
                        // printf( "%.2f", errors[count] );
                        // Visualize the flow in the frame
                        cv::line(this->frame, start, end, cv::Scalar(0, 255, 0) );
                }
                
                cv::circle(this->frame, start, 1, cv::Scalar(0, 255, 0) );

                count++;
            }
        }

    }

    this->prev_gray = gray;

    if( distance != 0 ){

        this->velocity.x = this->compute_velocity( this->flow.x, gray.cols, distance );
        this->velocity.x = this->compute_velocity( this->flow.x, gray.rows, distance );

        return this->velocity;
    }

    return this->flow;

}*/


cv::Point2f OpticalFlow::compute_sparse_flow( cv::Mat raw, float dt, float distance ){
    cv::Mat gray; // Output image for processing

    float xsum, ysum = 0;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, gray );

    // trackFeatures are made for stereo, so first two arguments is left and right image. 
    tracker->track_features(gray, this->features_current, this->status );

    if (!this->features_current.empty() && !this->features_previous.empty()) {
		//calculate pixel flow

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
                        cv::line(this->frame, cv::Point(features_previous[i].x,features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(255, 255, 255) );
                        cv::circle(this->frame, cv::Point(features_current[i].x,features_current[i].y), 2, cv::Scalar(255, 255, 255), -1 );

						count_confidense++; 
					} 
                }
            }

            if(count_confidense){
                // Calculate new pixel flow
                xsum = xsum_confidense;
                ysum = ysum_confidense;
                count = count_confidense;
            }
            
            this->flow.x = this->compute_pixel_velocity( xsum, count, dt);
            this->flow.y = this->compute_pixel_velocity( ysum, count, dt);
        }
	}

    this->features_previous = this->features_current;

    tracker->update_feature_status( this->status );

    if( distance != 0 ){

        this->velocity.x = this->compute_velocity( this->flow.x, gray.cols, distance );
        this->velocity.y = this->compute_velocity( this->flow.y, gray.rows, distance );

        return this->velocity;
    }
    return this->flow;
}

float OpticalFlow::compute_pixel_velocity( float sum, int count, float dt ){
    // Calculate pixels per second
    return (sum / (float)count) / dt;
}

float OpticalFlow::compute_velocity( float flow, int axis_length, float distance ){
    // Find focal length in pixels 
    int focal_length_px = (axis_length/2) / tan( this->fov/2 );

    // Scaling factor
    float pixel_per_length = (float)distance / focal_length_px;

    return flow * pixel_per_length;
}

cv::Mat OpticalFlow::get_frame( void ){
    return this->frame;
}

cv::Size OpticalFlow::get_size( void ){
    return this->size;
}

cv::Point2f OpticalFlow::get_flow( void ){
    return this->flow;
}