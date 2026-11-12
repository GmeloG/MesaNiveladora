// =====================================================================
//  LAMEC - Plataforma estabilizada de dois eixos (roll + pitch)
//  Servos MG996R, STM32 Nucleo-F767ZI, MPU6050 (GY-521)
//  Modelo paramétrico OpenSCAD - unidades em mm
//
//  Para exportar uma peça: mudar `part` (ou usar -D part="\"caixa\"")
//    "assembly" | "caixa" | "tampa" | "quadro" | "plataforma" | "tabuleiro"
// =====================================================================

part     = "assembly";
roll_a   = 0;     // só para visualização da montagem (graus)
pitch_a  = 0;
explode  = 0;     // vista explodida (mm)
show_ball = true; // mostrar tabuleiro + bola na montagem

$fn = 64;
tol = 0.3;        // folga geral de impressão

// ---------------- MG996R (VERIFICAR com paquímetro) ------------------
// Referencial do servo: veio em +Z, topo do corpo em z=0
sv_front   = 10.2;   // centro do veio -> extremidade do corpo (lado do veio)
sv_back    = 30.3;   // centro do veio -> extremidade oposta
sv_w       = 20.0;   // largura do corpo
sv_h       = 36.5;   // altura total do corpo (sem veio)
sv_fl_top  = 7.5;    // topo do corpo -> face de cima da aba
sv_fl_t    = 2.5;    // espessura da aba
sv_fl_len  = 54.5;   // comprimento total com abas
sv_hole_dx = 49.5;   // distância entre furos (comprimento)
sv_hole_dy = 10.0;   // distância entre furos (largura)
sv_hole_d  = 4.4;    // furos de fixação (parafuso M4 ou M3+anilha)
sv_c       = (sv_front - sv_back) / 2;   // centro do corpo em X

// Horn redondo (disco) que vem com o servo
horn_d      = 25.0;  // diâmetro do disco
horn_t      = 3.0;   // profundidade do rebaixo para o disco
horn_gap    = 3.0;   // topo do corpo -> face da peça acoplada
horn_hole_r = 8.5;   // raio dos furos de fixação do horn (VERIFICAR)
horn_hole_d = 2.0;   // furos para parafusos auto-roscantes M2
wall_face   = sv_fl_top + sv_fl_t;      // topo do corpo -> face da parede de fixação
stack       = wall_face + horn_gap;     // face da parede -> face acoplada (13)

// ---------------- Pivô (eixo oposto ao servo) -------------------------
piv_d   = 4.4;   // furo de rotação para parafuso M4
nut_af  = 7.0;   // porca M4 (entre faces)
nut_t   = 3.4;
piv_gap = 2.0;   // folga axial (anilha M4)

// ---------------- Gimbal ----------------------------------------------
H      = 70;     // altura dos eixos acima da tampa
P_w    = 90;     // lado da plataforma
P_t    = 4;      // espessura da plataforma
P_z    = 10;     // face inferior da plataforma acima do eixo
tab_t  = 7;      // espessura das patilhas da plataforma
hub_r  = 16;     // raio do cubo à volta de cada eixo

fr_xin = 53;     // face interior das paredes X do quadro
fr_xt  = 7;
fr_xout= fr_xin + fr_xt;                  // 60
fr_ym_face = -(P_w/2 + stack);            // face de apoio do servo de pitch (-58)
fr_yt  = 5;
fr_yp_in = P_w/2 + piv_gap;               // 47
fr_h   = 15;     // meia-altura das paredes do quadro

// ---------------- Caixa / tampa ---------------------------------------
bx_x0 = -108; bx_x1 = 82;  bx_y = 65;    // exterior (190 x 130)
bx_hz = 60;      // altura da caixa
bx_wt = 2.5;     // parede
bx_ft = 3;       // fundo
ld_t  = 4;       // espessura da tampa
roll_face_x = -fr_xout;                   // -60 (face acoplada roll)
tw_r_x1 = roll_face_x - stack;            // -73 face da torre do servo roll
tw_t   = 6;
tw_p_x0 = fr_xout + piv_gap;              // 62 torre do pivô
// =====================================================================

module nut_trap() cylinder(d = nut_af/cos(30) + 0.4, h = nut_t, $fn = 6);

// recorte do servo (referencial do servo), parede ocupa z<-wall_face
module servo_cut() {
    translate([-sv_back - tol, -sv_w/2 - tol, -60]) cube([sv_front + sv_back + 2*tol, sv_w + 2*tol, 80]);
    for (sx = [-1, 1], sy = [-1, 1])
        translate([sv_c + sx*sv_hole_dx/2, sy*sv_hole_dy/2, -60]) cylinder(d = sv_hole_d, h = 80, $fn = 24);
}

// rebaixo do horn na face acoplada (face em z=0, peça em z>0)
module horn_cut() {
    translate([0, 0, -1]) cylinder(d = horn_d + 2*tol, h = horn_t + 1);
    translate([0, 0, -1]) cylinder(d = 7, h = 40);          // acesso ao parafuso do veio
    for (a = [0:90:270]) rotate(a) translate([horn_hole_r, 0, -1]) cylinder(d = horn_hole_d, h = 40, $fn = 16);
}

// servo simplificado (só visualização)
module servo_dummy() {
    color("#222") {
        translate([-sv_back, -sv_w/2, -sv_h]) cube([sv_front + sv_back, sv_w, sv_h]);
        translate([sv_c - sv_fl_len/2, -sv_w/2, -wall_face]) cube([sv_fl_len, sv_w, sv_fl_t]);
    }
    color("#ddd") cylinder(d = 6, h = horn_gap);
    color("white") translate([0, 0, horn_gap]) cylinder(d = horn_d, h = horn_t);
}

// transformações servo -> mundo
module at_roll_servo()  // veio em +X
    translate([tw_r_x1 + wall_face, 0, H]) multmatrix([[0,0,1,0],[0,-1,0,0],[1,0,0,0]]) children();
module at_pitch_servo() // veio em +Y
    translate([0, fr_ym_face + wall_face, H]) multmatrix([[-1,0,0,0],[0,0,1,0],[0,1,0,0]]) children();

// ============================ CAIXA ==================================
module caixa() {
    L = bx_x1 - bx_x0; W = 2*bx_y;
    difference() {
        translate([bx_x0, -bx_y, -ld_t - bx_hz]) cube([L, W, bx_hz]);
        translate([bx_x0 + bx_wt, -bx_y + bx_wt, -ld_t - bx_hz + bx_ft]) cube([L - 2*bx_wt, W - 2*bx_wt, bx_hz]);
        // grelha de furos M3 no fundo para fixar a Nucleo / módulos (passo 10 mm)
        for (x = [-80:10:50], y = [-40:10:40])
            translate([x, y, -ld_t - bx_hz - 1]) cylinder(d = 3.2, h = bx_ft + 2, $fn = 16);
        // USB (ST-LINK) lado -X e USB user lado +X
        translate([bx_x0 - 1, -8, -ld_t - bx_hz + bx_ft + 6]) cube([bx_wt + 2, 16, 12]);
        translate([bx_x1 - bx_wt - 1, -8, -ld_t - bx_hz + bx_ft + 6]) cube([bx_wt + 2, 16, 12]);
        // jack de alimentação DC (Ø8) e interruptor (13x9) no lado +Y
        translate([40, bx_y + 1, -ld_t - bx_hz + 25]) rotate([90, 0, 0]) cylinder(d = 8, h = bx_wt + 2);
        translate([10, bx_y - bx_wt - 1, -ld_t - bx_hz + 20]) cube([13, bx_wt + 2, 9]);
        // ranhuras de ventilação no lado -Y
        for (x = [-80:10:40]) translate([x, -bx_y - 1, -ld_t - 40]) cube([4, bx_wt + 2, 25]);
    }
    // pilares dos cantos para os parafusos da tampa (furo Ø2.8 auto-roscante M3)
    for (p = corner_pts()) translate([p[0], p[1], -ld_t - bx_hz]) difference() {
        translate([-5, -5, 0]) cube([10, 10, bx_hz]);
        translate([0, 0, bx_hz - 16]) cylinder(d = 2.8, h = 20, $fn = 20);
    }
}
function corner_pts() = [[bx_x0 + 7, -bx_y + 7], [bx_x1 - 7, -bx_y + 7], [bx_x0 + 7, bx_y - 7], [bx_x1 - 7, bx_y - 7]];

// ======================= TAMPA + TORRES ==============================
module rib(len, h, t) // nervura triangular no plano XZ, espessura t em Y
    rotate([90, 0, 0]) translate([0, 0, -t/2]) linear_extrude(t) polygon([[0, 0], [len, 0], [0, h]]);

module tampa() {
    L = bx_x1 - bx_x0; W = 2*bx_y;
    difference() {
        union() {
            translate([bx_x0, -bx_y, -ld_t]) cube([L, W, ld_t]);
        }
        for (p = corner_pts()) translate([p[0], p[1], -ld_t - 6]) {
            cylinder(d = 3.4, h = 20, $fn = 20);
            translate([0, 0, 6 + ld_t - 2]) cylinder(d = 6.5, h = 3, $fn = 24);
        }
        translate([0, 0, -10]) cylinder(d = 18, h = 20);              // passagem cabos (servo pitch + MPU)
        translate([-92, 0, -10]) cylinder(d = 12, h = 20);            // passagem cabo servo roll
    }
    // torre do servo de roll
    difference() {
        union() {
            translate([tw_r_x1 - tw_t, -20, 0]) cube([tw_t, 40, H + 24]);
            for (y = [-17, 17]) {
                translate([tw_r_x1 - tw_t, y, 0]) mirror([1, 0, 0]) rib(22, 36, 6);
                translate([tw_r_x1, y, 0]) rib(8, 30, 6);
            }
        }
        at_roll_servo() servo_cut();
    }
    // torre do pivô de roll
    difference() {
        union() {
            translate([tw_p_x0, -15, 0]) cube([tw_t, 30, H + 14]);
            hull() { translate([tw_p_x0, -15, 0]) cube([tw_t, 30, 1]); }
            for (y = [-12, 12]) translate([tw_p_x0 + tw_t, y, 0]) rib(13, 45, 6);
        }
        translate([tw_p_x0 - 1, 0, H]) rotate([0, 90, 0]) cylinder(d = piv_d, h = tw_t + 2, $fn = 32);
    }
}

// ======================= QUADRO (eixo roll) ==========================
module quadro() {
    y0 = fr_ym_face - fr_yt;       // -63
    y1 = fr_yp_in + fr_yt;         // 52
    difference() {
        union() {
            translate([-fr_xout, y0, H - fr_h]) cube([fr_xt, y1 - y0, 2*fr_h]);   // parede -X
            translate([fr_xin,  y0, H - fr_h]) cube([fr_xt, y1 - y0, 2*fr_h]);   // parede +X
            translate([-fr_xout, y0, H - fr_h]) cube([2*fr_xout, fr_yt, 2*fr_h]); // parede -Y (servo pitch)
            translate([-fr_xout, fr_yp_in, H - fr_h]) cube([2*fr_xout, fr_yt, 2*fr_h]); // parede +Y (pivô)
            // reforços nos cantos interiores
            for (sx = [-1, 1], yy = [[y0 + fr_yt, 1], [fr_yp_in, -1]])
                translate([sx*fr_xin, yy[0], H - fr_h]) rotate(sx < 0 ? (yy[1] > 0 ? 0 : -90) : (yy[1] > 0 ? 90 : 180))
                    linear_extrude(2*fr_h) polygon([[0, 0], [8, 0], [0, 8]]);
        }
        // acoplamento ao horn do servo roll (face exterior -X)
        translate([-fr_xout, 0, H]) rotate([0, 90, 0]) horn_cut();
        // pivô roll: furo + porca na face interior +X
        translate([fr_xin - 1, 0, H]) rotate([0, 90, 0]) cylinder(d = piv_d - 0.4, h = fr_xt + 2, $fn = 32);
        translate([fr_xin - 0.01, 0, H]) rotate([0, 90, 0]) rotate(30) nut_trap();
        // servo de pitch
        at_pitch_servo() servo_cut();
        // pivô pitch
        translate([0, fr_yp_in - 1, H]) rotate([-90, 0, 0]) cylinder(d = piv_d, h = fr_yt + 2, $fn = 32);
        // passagem de braçadeiras para o cabo do servo
        translate([-40, y0 - 1, H - 12]) cube([4, fr_yt + 2, 3]);
    }
}

// ======================= PLATAFORMA (eixo pitch) =====================
module plat_tab(y_out, dir) { // dir=+1 patilha cresce para +Y a partir de y_out
    translate([0, y_out + (dir > 0 ? 0 : -tab_t), 0])
    intersection() {
    translate([-50, -1, 0]) cube([100, tab_t + 2, H + P_z + P_t]);
    hull() {
        translate([0, 0, H]) rotate([-90, 0, 0]) cylinder(r = hub_r, h = tab_t);
        translate([-28, 0, H + P_z - 1]) cube([56, tab_t, 1]);
    }}
}
module plataforma() {
    difference() {
        union() {
            translate([-P_w/2, -P_w/2, H + P_z]) cube([P_w, P_w, P_t]);
            plat_tab(-P_w/2, 1);             // lado do servo (face exterior em y=-45)
            plat_tab( P_w/2, -1);            // lado do pivô
        }
        // horn do servo de pitch na face exterior -Y
        translate([0, -P_w/2, H]) rotate([-90, 0, 0]) horn_cut();
        // pivô pitch: furo + porca na face interior
        translate([0, P_w/2 + 1, H]) rotate([90, 0, 0]) cylinder(d = piv_d - 0.4, h = tab_t + 2, $fn = 32);
        translate([0, P_w/2 - tab_t + 0.01, H]) rotate([90, 0, 0]) rotate(30) nut_trap();
        // MPU6050 (GY-521) ao centro: rebaixo 0.6 mm + 2 furos M2.5 (VERIFICAR)
        translate([-21.5/2, -16.5/2, H + P_z + P_t - 0.6]) cube([21.5, 16.5, 1]);
        for (x = [-7.62, 7.62]) translate([x, 16.5/2 - 2.5, H + P_z - 1]) cylinder(d = 2.8, h = P_t + 2, $fn = 20);
        translate([-6, -16.5/2 - 8, H + P_z - 1]) cube([12, 4, P_t + 2]);   // passagem do cabo do MPU
        // furos M3 nos cantos para fixar carga
        for (sx = [-1, 1], sy = [-1, 1]) translate([sx*(P_w/2 - 6), sy*(P_w/2 - 6), H + P_z - 1]) cylinder(d = 3.4, h = P_t + 2, $fn = 20);
        // marcação dos eixos (X = roll)
        translate([P_w/2 - 14, -0.5, H + P_z + P_t - 0.5]) cube([10, 1, 1]);
    }
}

// ======================= TABULEIRO (bola) ============================
// 88x88 mm, assente na plataforma, 4x M3x12 cabeça de embeber + porca.
// Livre até ±20° em roll/pitch (limitar no software).
tb_w    = 88;   // lado do tabuleiro (cabe dentro do quadro)
tb_ch   = 5;    // chanfro dos cantos (folga para os reforços do quadro)
tb_t    = 4;    // espessura da base
rim_h   = 6;    // altura do rebordo
rim_t   = 2;
tb_z    = H + P_z + P_t;             // assente direto na plataforma
hole_xy = P_w/2 - 6;                 // furos M3 da plataforma (±39)

module tb_outline(d = 0) offset(delta = -d) offset(delta = tb_ch, chamfer = true) square(tb_w - 2*tb_ch, center = true);
module tabuleiro() {
    difference() {
        translate([0, 0, tb_z]) union() {
            linear_extrude(tb_t) tb_outline();
            linear_extrude(tb_t + rim_h) difference() { tb_outline(); tb_outline(rim_t); }   // rebordo
        }
        // furos escareados M3 (parafuso de cabeça de embeber, de cima) + acesso à cabeça através do rebordo
        for (sx = [-1, 1], sy = [-1, 1]) translate([sx*hole_xy, sy*hole_xy, tb_z - 1]) {
            cylinder(d = 3.4, h = tb_t + 2, $fn = 24);
            translate([0, 0, 1 + tb_t - 1.8]) cylinder(d1 = 3.4, d2 = 6.6, h = 1.81, $fn = 24);
            translate([0, 0, 1 + tb_t]) cylinder(d = 6.6, h = rim_h + 1, $fn = 24);
        }
        // marca do centro e do eixo X (0.4 mm)
        translate([0, 0, tb_z + tb_t - 0.4]) difference() { cylinder(d = 12, h = 1); cylinder(d = 10, h = 1); }
        translate([tb_w/2 - rim_t - 20, -0.5, tb_z + tb_t - 0.4]) cube([12, 1, 1]);
    }
}

// ============================ MONTAGEM ===============================
module rot_roll() translate([0, 0, H]) rotate([roll_a, 0, 0]) translate([0, 0, -H]) children();
module rot_pitch() translate([0, 0, H]) rotate([0, pitch_a, 0]) translate([0, 0, -H]) children();

module assembly() {
    color("#4a7bd1") translate([0, 0, -explode]) caixa();
    color("#e0e0e0") tampa();
    at_roll_servo() servo_dummy();
    rot_roll() {
        color("#f28c28") translate([0, 0, explode]) quadro();
        at_pitch_servo() servo_dummy();
        rot_pitch() {
            color("#3fae5a") translate([0, 0, 2*explode]) plataforma();
            if (show_ball) {
                color("#d94a4a") translate([0, 0, 3*explode]) tabuleiro();
                color("white") translate([10, -15, tb_z + tb_t + 20 + 3*explode]) sphere(d = 40);
            }
        }
    }
}

// ===================== peças orientadas p/ impressão =================
if (part == "assembly") assembly();
else if (part == "caixa")      translate([0, 0, ld_t + bx_hz]) caixa();
else if (part == "tampa")      translate([0, 0, ld_t]) tampa();          // imprimir com torres para cima
else if (part == "quadro")     translate([0, 0, fr_h - H]) quadro();     // paredes na vertical
else if (part == "plataforma") rotate([180, 0, 0]) translate([0, 0, -(H + P_z + P_t)]) plataforma(); // placa na mesa
else if (part == "tabuleiro")  translate([0, 0, -tb_z]) tabuleiro();               // base na mesa
