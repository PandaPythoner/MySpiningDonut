#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <cmath>


typedef double flt;


class Vec{
public:
    flt x, y, z;

    Vec() : x(0), y(0), z(0) {}
    Vec(flt x, flt y, flt z) : x(x), y(y), z(z) {}

    Vec operator-(){
        return Vec(-x, -y, -z);
    }

    Vec normalize(){
        flt d = sqrt(x * x + y * y + z * z);
        return Vec(x / d, y / d, z / d);
    }
};


Vec operator+(const Vec& a, const Vec& b){
    return Vec(a.x + b.x, a.y + b.y, a.z + b.z);
}


Vec operator-(const Vec& a, const Vec& b){
    return Vec(a.x - b.x, a.y - b.y, a.z - b.z);
}


Vec operator*(const Vec& a, flt x){
    return Vec(a.x * x, a.y * x, a.z * x);
}

Vec operator*(flt x, const Vec& a){
    return Vec(a.x * x, a.y * x, a.z * x);
}


flt operator*(const Vec& a, const Vec& b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}


Vec operator%(const Vec& a, const Vec& b){
    return Vec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}


class Quaternion{
public:
    flt t;
    Vec v;

    Quaternion() : t(0), v() {}
    Quaternion(const flt& t) : t(t), v() {}
    Quaternion(const Vec& v) : t(0), v(v) {}
    Quaternion(const flt& t, const Vec& v) : t(t), v(v) {}
    Quaternion(const flt& t, const flt& x, const flt& y, const flt& z) : t(t), v(x, y, z) {}

    Quaternion inv(){
        return Quaternion(t, -v);
    }
};



Quaternion operator+(const Quaternion& a, const Quaternion& b){
    return Quaternion(a.t + b.t, a.v + b.v);
}


Quaternion operator-(const Quaternion& a, const Quaternion& b){
    return Quaternion(a.t - b.t, a.v - b.v);
}


Quaternion operator*(const Quaternion& a, const Quaternion &b){
    return Quaternion(a.t * b.t - a.v * b.v, a.t * b.v + b.t * a.v + (a.v % b.v));
}


class Donut{
public:
    flt PHI = atan2(0, -1);
    const flt inf = 1e40;

    Quaternion rotor_left, rotor_right;
    int screen_x, screen_y;
    int screen_center_x, screen_center_y;
    flt R;
    flt r;
    flt D;
    flt d;

    int iters_big_circle, iters_small_circle;

    Vec rotation_axis = Vec(2, 3, 1).normalize();

    Vec light_direction = Vec(4, 4, -1).normalize();

    std::string brightness_string = ".,-~:;=!*#$@";

    flt y_factor;

    Donut(int screen_x = 100, int screen_y = 40, flt R = 100, flt r = 30, flt D = 500, flt d = 90,
        int iters_big_circle = 200, int iters_small_circle = 200, flt y_factor=0.6) 
        : rotor_left(1), rotor_right(1), screen_x(screen_x), screen_y(screen_y), 
        screen_center_x(screen_x / 2), screen_center_y(screen_y / 2),
        R(R), r(r), D(D), d(d), 
        iters_big_circle(iters_big_circle), iters_small_circle(iters_small_circle),
        y_factor(y_factor) {
        
    }

    std::vector<std::string> render(){
        std::vector<std::vector<std::pair<flt, char>>> vrs(screen_y, std::vector<std::pair<flt, char>>(screen_x, {inf, ' '}));
        for(int i = 0; i < iters_big_circle; i += 1){
            flt angi = (2 * PHI * i) / iters_big_circle;
            Vec veci(cos(angi), sin(angi), 0);
            for(int j = 0; j < iters_small_circle; j += 1){
                flt angj = (2 * PHI * j) / iters_small_circle;
                Vec vecj = cos(angj) * veci + Vec(0, 0, sin(angj));

                Vec normal_vec = (rotor_left * Quaternion(vecj) * rotor_right).v;

                flt brightness = (light_direction * normal_vec + 1) / 2;


                Vec pos = veci * R + vecj * r;

                pos = (rotor_left * Quaternion(pos) * rotor_right).v;
                flt md = D + pos.z;
                if(abs(md) < 1){
                    continue;
                }
                int screen_pos_x = round(pos.x * d / md) + screen_center_x;
                int screen_pos_y = screen_y - (round(pos.y * d / md * y_factor) + screen_center_y);
                char c = brightness_string[(int)(brightness * brightness_string.size())];
                if(0 <= screen_pos_x && screen_pos_x < screen_x && 0 <= screen_pos_y && screen_pos_y < screen_y){
                    vrs[screen_pos_y][screen_pos_x] = min(vrs[screen_pos_y][screen_pos_x], std::make_pair(pos.z, c));
                }
            }
        }
        std::vector<std::string> rs(screen_y, std::string(screen_x, ' '));
        for(int i = 0; i < screen_y; i += 1){
            for(int j = 0; j < screen_x; j += 1){
                rs[i][j] = vrs[i][j].second;
            }
        }
        return rs;
    }

    void rotate(flt rotation_angle = 0.12){
        Quaternion rotation_quaternion = Quaternion(cos(rotation_angle), sin(rotation_angle) * rotation_axis);
        rotor_left = rotation_quaternion * rotor_left;
        rotor_right = rotor_right * rotation_quaternion.inv();
    }
};


void sleep(long long x){
    std::this_thread::sleep_for(std::chrono::milliseconds(x));
}





int main(){
    Donut donut;
    while(1){
        donut.rotate();
        auto rndrd = donut.render();
        // system("clear");
        for(auto s: rndrd){
            std::cout << s << "\n";
        }
        std::cout << std::endl;
        sleep(100);
    }
    return 0;
}