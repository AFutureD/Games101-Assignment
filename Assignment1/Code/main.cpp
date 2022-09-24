#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f Rodrigues_Rotation_Formula(Eigen::Vector3f pos, float degree) {
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N {
        {0, -pos[2], pos[1]},
        {pos[2], 0, -pos[0]},
        {-pos[1], pos[0], 0},
    };
    Eigen::Matrix3f M;
    M = cos(degree) * I + (1 - cos(degree)) * pos * pos.transpose() + sin(degree) * N;

    Eigen::Matrix4f R {
        {M(0,0), M(0, 1), M(0, 2), 0},
        {M(1,0), M(1, 1), M(1, 2), 0},
        {M(2,0), M(2, 1), M(2, 2), 0},
        {0, 0, 0, 1},
    };
    return R;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Vector3f up_at_y_pos {0,1,0}; // In the assigment, this is the default value.
    Eigen::Vector3f camera_pos = eye_pos.normalized();
    Eigen::Matrix4f translate {
        {1, 0, 0, -eye_pos[0]},
        {0, 1, 0, -eye_pos[1]},
        {0, 0, 1, -eye_pos[2]},
        {0, 0, 0, 1}
    };

    Eigen::Vector3f gxt = up_at_y_pos.cross(camera_pos);
    Eigen::Matrix4f rotation_inverse {
        {gxt[0], up_at_y_pos[0], camera_pos[0], 0},
        {gxt[1], up_at_y_pos[1], camera_pos[1], 0},
        {gxt[2], up_at_y_pos[2], camera_pos[2], 0},
        {0, 0, 0, 1}
    };
    Eigen::Matrix4f rotation = rotation_inverse.transpose(); // R_{view}^{-1} to R_view

    view = rotation * translate * view;
    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float TO_DEGREE = M_PI / 180.0f;
    float rad = rotation_angle * TO_DEGREE;

    Eigen::Matrix4f translate {
        {cos(rad), -sin(rad), 0, 0},
        {sin(rad), cos(rad), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };
    return translate * model;
}

Eigen::Matrix4f get_model_matrix_complate(float roll, float yaw, float pitch)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float TO_DEGREE = M_PI / 180.0f;
    float rad_z = roll * TO_DEGREE;
    float rad_y = yaw * TO_DEGREE;
    float rad_x = pitch * TO_DEGREE;

    Eigen::Matrix4f R_z = Rodrigues_Rotation_Formula({0,0,1},rad_z);
    Eigen::Matrix4f R_y = Rodrigues_Rotation_Formula({0, 1, 0},rad_y);
    Eigen::Matrix4f R_x = Rodrigues_Rotation_Formula({1, 0, 0},rad_x);

    return R_x * R_y * R_z * model;
}

/// 使用给定的参数逐个元素地构建透视投影矩阵并返回 该矩阵。
Eigen::Matrix4f get_projection_matrix(
    float eye_fov,
    float aspect_ratio,
    float zNear,
    float zFar
) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.

    float half_rad = eye_fov * M_PI / 180.0f;
    float top      = zNear * tan(half_rad);
    float bottom   = -top;
    float right    = top * aspect_ratio;
    float left     = -right;

    Eigen::Matrix4f M_ortho_scale {
        {2 / (right - left), 0, 0, 0},
        {0, 2 / (top - bottom), 0, 0},
        {0, 0, 2 / (zNear - zFar), 0},
        {0, 0, 0, 1},
    };

    Eigen::Matrix4f M_ortho_move {
        {1, 0, 0, -(right + left) / 2},
        {0, 1, 0, -(top + bottom) / 2},
        {0, 0, 1, -(zNear + zFar) / 2},
        {0, 0, 0, 1},
    };

    Eigen::Matrix4f M_ortho = M_ortho_scale * M_ortho_move;

    Eigen::Matrix4f M_persp_2_ortho {
        {zNear,     0,            0,             0},
        {    0, zNear,            0,             0},
        {    0,     0, zNear + zFar, -zNear * zFar},
        {    0,     0,            1,             0},
    };

    projection = M_ortho * M_persp_2_ortho;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0; // roll
    float yaw = 0;
    float pitch = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    // The human eye position.
    Eigen::Vector3f eye_pos = {0, 0, 5};

    // Define the Tringle
    std::vector<Eigen::Vector3f> pos {
        {2, 0, -2},
        {0, 2, -2},
        {-2, 0, -2}
    };

    // The order of the points.
    std::vector<Eigen::Vector3i> ind {{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // The MVP Operations.

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix_complate(angle, yaw, pitch));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(100);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        } else if (key == 'd') {
            angle -= 10;
        } else if (key == 'w') {
            pitch += 10;
        } else if (key == 's') {
            pitch -= 10;
        } else if (key == 'q') {
            yaw += 10;
        } else if (key == 'e') {
            yaw -= 10;
        }
    }

    return 0;
}
