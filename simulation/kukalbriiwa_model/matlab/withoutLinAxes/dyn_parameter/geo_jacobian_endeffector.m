function tmpreturn = geo_jacobian_endeffector(q, param)
  m = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0;];
  t1 = q(5);
  t2 = cos(t1);
  t3 = q(2);
  t4 = sin(t3);
  t5 = t4 * t2;
  t6 = sin(param.roll);
  t8 = sin(param.pitch);
  t9 = param.d7 + param.d8;
  t11 = q(6);
  t12 = sin(t11);
  t15 = cos(t3);
  t16 = q(3);
  t17 = cos(t16);
  t18 = t17 * t15;
  t19 = t8 * t6;
  t21 = sin(t16);
  t22 = cos(param.pitch);
  t25 = cos(t11);
  t27 = t25 * t9 + param.d5 + param.d6;
  t30 = q(4);
  t31 = sin(t30);
  t33 = cos(t30);
  t34 = t2 * t33;
  t36 = sin(t1);
  t37 = t36 * t17;
  t38 = -t21 * t34 - t37;
  t40 = t15 * t6;
  t41 = t33 * t17;
  t43 = t21 * t36;
  t44 = t2 * t41 - t43;
  t50 = t33 * t27;
  t51 = t50 + param.d3 + param.d4;
  t52 = t4 * t51;
  t55 = q(1);
  t56 = cos(t55);
  t68 = t44 * t15;
  t70 = -t38;
  t78 = sin(t55);
  t80 = t56 * (t31 * (t12 * t9 * t8 * t6 * t5 - t27 * (t18 * t19 - t21 * t22)) + t12 * (t40 * t44 * t8 + t22 * t38) * t9 + t19 * t52) - t78 * (t31 * (t12 * t9 * t22 * t5 - t27 * (t21 * t6 * t8 + t18 * t22)) + t12 * (t19 * t70 + t22 * t68) * t9 + t4 * t51 * t22);
  t81 = cos(param.yaw);
  t83 = cos(param.roll);
  t84 = t12 * t9;
  t86 = t17 * t27;
  t88 = -t15 * t86 + t5 * t84;
  t93 = t12 * t15 * t44 * t9 + t31 * t88 + t52;
  t95 = t27 * t21;
  t98 = -t31 * t95 + t70 * t84;
  t100 = t56 * t93 - t78 * t98;
  t101 = t100 * t83;
  t102 = sin(param.yaw);
  m(1,1) = -t102 * t101 + t80 * t81;
  t104 = t2 * t12;
  t108 = -t104 * t33 * t9 + t27 * t31;
  t110 = t84 * t43;
  t115 = t104 * t31 * t9 + param.d3 + param.d4 + t50;
  t116 = t78 * t115;
  t120 = t22 * t56;
  t125 = -t108;
  t127 = t125 * t17 - t110;
  t128 = t78 * t127;
  t135 = t15 * (t8 * (t83 * (t108 * t17 + t110) + t6 * t116) + t115 * t120) - (t8 * (t115 * t83 + t128 * t6) + t127 * t120) * t4;
  t145 = t15 * (t116 * t83 + t127 * t6) + (t115 * t6 - t128 * t83) * t4;
  m(1,2) = -t102 * t145 + t135 * t81;
  t148 = t83 * t4;
  t149 = t40 * t78 - t148;
  t155 = t33 * t15;
  t156 = t78 * t2;
  t159 = t155 * t156 + t36 * t56;
  t164 = t15 * t56;
  t166 = t36 * t78;
  t167 = t164 * t34 - t166;
  t177 = t56 * t6 * t8 - t22 * t78;
  t180 = t36 * t4;
  t182 = t33 * t56;
  t184 = t78 * t15;
  t186 = t182 * t2 - t184 * t36;
  t192 = t164 * t36 + t34 * t78;
  t199 = t21 * (t31 * (t120 * t15 + t149 * t8) * t27 - t12 * t9 * (t8 * (-t148 * t34 + t159 * t6) + t167 * t22)) - (t31 * t177 * t27 + t12 * t9 * (t8 * (-t180 * t83 - t186 * t6) + t192 * t22)) * t17;
  t201 = t83 * t78;
  t203 = t6 * t4;
  t224 = t21 * (-t31 * t27 * (t15 * t201 + t203) + t12 * (t159 * t83 + t203 * t34) * t9) + (t31 * t83 * t27 * t56 - t12 * t9 * (-t180 * t6 + t186 * t83)) * t17;
  m(1,3) = t102 * t224 + t199 * t81;
  t226 = t2 * t15;
  t229 = t226 * t84 + t4 * t86;
  t230 = t83 * t229;
  t233 = t12 * t9 * t4;
  t235 = t56 * t95;
  t236 = -t156 * t233 + t184 * t86 + t235;
  t240 = t56 * t17;
  t241 = t15 * t27;
  t243 = t78 * t21;
  t244 = t27 * t243;
  t245 = t2 * t56;
  t251 = t2 * t17;
  t253 = -t233 * t251 + t241;
  t254 = t83 * t253;
  t256 = t15 * t84;
  t258 = t27 * t4;
  t260 = t21 * t2;
  t262 = t56 * t84 * t260;
  t263 = t251 * t256 * t78 + t258 * t78 + t262;
  t269 = t12 * t21;
  t271 = t9 * t269 * t156;
  t272 = t4 * t56;
  t278 = t33 * (t8 * (-t236 * t6 + t230) - (-t233 * t245 + t240 * t241 - t244) * t22) - (t8 * (t263 * t6 + t254) + t22 * (t2 * t240 * t256 + t27 * t272 - t271)) * t31;
  t289 = t33 * (t229 * t6 + t236 * t83) - (t253 * t6 - t263 * t83) * t31;
  m(1,4) = t102 * t289 + t278 * t81;
  t292 = t15 * t31;
  t293 = -t4 * t41 + t292;
  t294 = t83 * t293;
  t295 = t78 * t33;
  t297 = t21 * t33;
  t298 = t56 * t297;
  t299 = t4 * t78;
  t300 = t31 * t299;
  t301 = t18 * t295 + t298 + t300;
  t306 = t21 * t295;
  t307 = t31 * t272;
  t312 = t21 * t4;
  t313 = t83 * t312;
  t315 = -t15 * t243 + t240;
  t317 = -t315 * t6 - t313;
  t321 = t164 * t21 + t17 * t78;
  t323 = t22 * t321 + t317 * t8;
  t325 = t36 * (t8 * (t301 * t6 + t294) + (t18 * t182 - t306 + t307) * t22) + t323 * t2;
  t334 = -t312 * t6 + t315 * t83;
  t336 = t36 * (t293 * t6 - t301 * t83) + t334 * t2;
  m(1,5) = -t12 * t9 * (t102 * t336 + t325 * t81);
  t343 = t226 * t31 - t4 * t44;
  t344 = t83 * t343;
  t350 = t15 * t44 * t78 + t36 * t240 + (t298 + t300) * t2;
  t365 = -t17 * t31 * t4 - t155;
  t366 = t83 * t365;
  t370 = t56 * t31 * t21;
  t372 = t18 * t31 * t78 - t299 * t33 + t370;
  t379 = t243 * t31;
  t384 = t25 * (t8 * (t350 * t6 + t344) + (t15 * t44 * t56 - t17 * t166 + (t307 - t306) * t2) * t22) + (t8 * (t372 * t6 + t366) + (t164 * t17 * t31 - t272 * t33 - t379) * t22) * t12;
  t391 = -t365;
  t395 = t25 * (-t343 * t6 + t350 * t83) + t12 * (t372 * t83 + t391 * t6);
  m(1,6) = (-t102 * t395 + t384 * t81) * t9;
  m(1,7) = 0.0e0;
  m(2,1) = t101 * t81 + t102 * t80;
  m(2,2) = t102 * t135 + t145 * t81;
  m(2,3) = t102 * t199 - t224 * t81;
  m(2,4) = t102 * t278 - t289 * t81;
  m(2,5) = -(t102 * t325 - t336 * t81) * t84;
  m(2,6) = (t102 * t384 + t395 * t81) * t9;
  m(2,7) = 0.0e0;
  m(3,1) = t22 * t6 * t100 + t8 * (t56 * t98 + t78 * t93);
  t425 = t9 * t6;
  t428 = t9 * t83;
  t432 = param.d3 + param.d4;
  m(3,2) = t22 * (t15 * (t12 * t156 * t31 * t425 + t27 * t33 * t6 * t78 + t12 * t428 * t43 - t125 * t17 * t83 + t432 * t6 * t78) - t4 * (-t12 * t243 * t36 * t425 + t125 * t17 * t6 * t78 + t104 * t31 * t428 + t27 * t33 * t83 + t432 * t83)) - (t115 * t15 - t127 * t4) * t56 * t8;
  m(3,3) = -(t21 * (t104 * t149 * t33 * t9 + t12 * t36 * t425 * t56 - t149 * t27 * t31) + t17 * (t12 * t149 * t36 * t9 - t12 * t245 * t33 * t425 + t27 * t31 * t56 * t6)) * t22 - t8 * t27 * t31 * t321 + t9 * (t167 * t21 + t17 * t192) * t8 * t12;
  t488 = -t88;
  t495 = t251 * t256 + t258;
  m(3,4) = t22 * (t33 * (t6 * (-t488 * t78 - t235) + t230) - (t6 * (t495 * t78 + t262) + t254) * t31) + (t33 * (t488 * t56 - t244) + t31 * (t495 * t56 - t271)) * t8;
  t512 = t31 * t4;
  t513 = t15 * t41 + t512;
  m(3,5) = (t22 * (t36 * (t6 * (-t513 * t78 - t298) - t294) - t2 * t317) + t8 * (t36 * (t513 * t56 - t306) + t321 * t2)) * t84;
  t532 = t31 * t5 + t68;
  t539 = t4 * t33;
  t540 = t18 * t31 - t539;
  t542 = t540 * t78 + t370;
  t545 = -t391 * t83 + t542 * t6;
  m(3,6) = -t9 * (t22 * (t25 * (t6 * (t38 * t56 - t532 * t78) - t344) - t12 * t545) + t8 * (t25 * (t532 * t56 - t70 * t78) + (t540 * t56 - t379) * t12));
  m(3,7) = 0.0e0;
  m(4,1) = t8 * t81 * t83 + t102 * t6;
  m(4,2) = -t102 * t56 * t83 + t177 * t81;
  t568 = t15 * t83 + t299 * t6;
  t572 = t22 * t4 * t56 + t568 * t8;
  t575 = -t201 * t4 + t40;
  m(4,3) = t102 * t575 + t572 * t81;
  m(4,4) = t102 * t334 + t323 * t81;
  t581 = -t540;
  t583 = t56 * t581 + t379;
  t585 = t22 * t583 - t545 * t8;
  t589 = -t365 * t6 + t542 * t83;
  m(4,5) = t102 * t589 + t585 * t81;
  t594 = t260 * t4 - t292 * t36 + t37 * t539;
  t597 = t36 * t41 + t260;
  t599 = t36 * t512;
  t603 = t297 * t36 - t251;
  t605 = (t15 * t597 + t599) * t78 + t56 * t603;
  t612 = -t15 * t597 - t599;
  t614 = t56 * t612 + t603 * t78;
  t616 = t8 * (t594 * t83 - t6 * t605) + t614 * t22;
  t619 = -t594;
  t621 = -t6 * t619 + t605 * t83;
  m(4,6) = t102 * t621 + t616 * t81;
  t625 = t104 * t31 + t25 * t33;
  t629 = -t12 * t34 + t25 * t31;
  t631 = t17 * t4 * t629;
  t633 = t4 * t12 * t43;
  t637 = t36 * t12 * t17;
  t639 = t21 * t629 - t637;
  t642 = t36 * t269;
  t645 = t625 * t4;
  t648 = t56 * t639 + t78 * (t15 * (t17 * t629 + t642) - t645);
  t653 = -t629;
  t657 = t15 * (t17 * t653 - t642) + t645;
  t659 = t56 * t657 + t639 * t78;
  t661 = t8 * (t83 * (t15 * t625 + t631 + t633) - t6 * t648) + t659 * t22;
  t666 = -t15 * t625 - t631 - t633;
  t668 = -t6 * t666 + t648 * t83;
  m(4,7) = t102 * t668 + t661 * t81;
  m(5,1) = t102 * t8 * t83 - t6 * t81;
  m(5,2) = t56 * t81 * t83 + t102 * t177;
  m(5,3) = t102 * t572 - t575 * t81;
  m(5,4) = t102 * t323 - t334 * t81;
  m(5,5) = t102 * t585 - t589 * t81;
  m(5,6) = t102 * t616 - t621 * t81;
  m(5,7) = t102 * t661 - t668 * t81;
  m(6,1) = t83 * t22;
  m(6,2) = t22 * t56 * t6 + t78 * t8;
  m(6,3) = -t4 * t56 * t8 + t22 * t568;
  m(6,4) = t22 * (-t315 * t6 - t313) - t321 * t8;
  m(6,5) = t22 * (t6 * (t581 * t78 - t370) - t366) - t583 * t8;
  m(6,6) = t22 * (t6 * (-t56 * t603 + t612 * t78) - t83 * t619) - t614 * t8;
  m(6,7) = t22 * (t6 * (t56 * (t21 * t653 + t637) + t78 * t657) - t83 * t666) - t659 * t8;
  tmpreturn = m;
