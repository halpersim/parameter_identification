function tmpreturn = inertia_matrix(q, param)
  m = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0;];
  t1 = q(4);
  t2 = sin(t1);
  t3 = param.m7 * param.sp7x;
  t4 = q(7);
  t5 = cos(t4);
  t7 = param.m7 * param.sp7y;
  t8 = sin(t4);
  t9 = t8 * t7;
  t10 = param.m6 * param.sp6x;
  t11 = -t3 * t5 + t10 + t9;
  t13 = param.d3 + param.d4;
  t14 = t13 * t11 * t2;
  t15 = q(5);
  t16 = sin(t15);
  t17 = param.sp7y * t3;
  t18 = -t17 + param.I7xy;
  t19 = 0.2e1 * t18;
  t20 = t5 ^ 2;
  t21 = t20 * t19;
  t22 = param.sp7x ^ 2;
  t23 = param.sp7y ^ 2;
  t24 = t22 - t23;
  t26 = param.m7 * t24 - param.I7xx + param.I7yy;
  t27 = t26 * t8;
  t28 = t5 * t27;
  t29 = param.sp6z * t10;
  t30 = t21 - t28 + t17 + t29 - param.I6xz - param.I7xy;
  t31 = t30 * t16;
  t34 = q(6);
  t35 = cos(t34);
  t37 = -param.d7 - param.sp7z;
  t39 = param.m6 * param.sp6y;
  t41 = sin(t34);
  t42 = t41 * (param.m7 * t37 - t39);
  t43 = param.m5 * param.sp5x;
  t44 = t42 + t43;
  t45 = t44 * t13;
  t46 = t2 * t45;
  t47 = 0.2e1 * t46;
  t48 = -t37;
  t50 = param.m7 * t48 * param.sp7y;
  t51 = t50 - param.I7yz;
  t53 = t5 * t51 * t41;
  t55 = param.m7 * t48 * param.sp7x;
  t56 = t55 - param.I7xz;
  t57 = t8 * t56;
  t58 = param.sp6z * t39;
  t59 = t57 + t58 - param.I6yz;
  t60 = t41 * t59;
  t62 = param.sp5x * param.sp5y * param.m5;
  t64 = t16 * (t53 + t60 + t62 - param.I5xy);
  t67 = cos(t15);
  t69 = t41 * param.sp7x;
  t70 = param.d5 + param.d6;
  t71 = t70 * param.m7;
  t74 = 0.2e1 * t5 * t71 * t69;
  t75 = (param.sp4x ^ 2);
  t76 = (param.sp4y ^ 2);
  t77 = (t75 + t76);
  t78 = (t77 * param.m4);
  t79 = param.d4 ^ 2;
  t80 = param.m4 * t79;
  t81 = param.d5 ^ 2;
  t83 = 2 * param.d5 * param.d6;
  t84 = param.d6 ^ 2;
  t85 = t13 ^ 2;
  t88 = param.sp1x ^ 2;
  t89 = param.sp1y ^ 2;
  t92 = param.m3 + param.m4;
  t93 = param.d3 ^ 2;
  t94 = t93 * t92;
  t95 = (param.d4 * param.m4);
  t98 = 2 * param.m3 * param.sp3z + 2 * t95;
  t99 = param.d3 * t98;
  t100 = (param.sp3x ^ 2);
  t101 = param.sp3z ^ 2;
  t104 = param.sp2y ^ 2;
  t105 = param.sp2z ^ 2;
  t109 = 2 * param.d3 * param.d4;
  t110 = param.sp6x ^ 2;
  t111 = param.sp6z ^ 2;
  t115 = t5 * t16 * t7;
  t116 = t8 * t3;
  t117 = param.m5 * param.sp5y;
  t118 = param.m6 * param.sp6z;
  t119 = -t116 + t117 - t118;
  t121 = param.m4 * param.sp4x;
  t123 = (t119 * t16 - t115 + t121) * t13;
  t125 = 0.2e1 * t2 * t123;
  t126 = q(3);
  t127 = sin(t126);
  t128 = -t18;
  t129 = 0.4e1 * t128;
  t130 = t20 * t129;
  t131 = 0.2e1 * t28;
  t132 = 0.2e1 * t29;
  t133 = 0.2e1 * t17;
  t134 = 0.2e1 * param.I6xz;
  t135 = 0.2e1 * param.I7xy;
  t139 = 0.2e1 * t56;
  t144 = t41 * (t139 * t8 - 0.2e1 * param.I6yz + 0.2e1 * t58);
  t145 = 0.2e1 * t62;
  t146 = 0.2e1 * param.I5xy;
  t148 = t67 ^ 2;
  t150 = t20 * t26;
  t151 = t128 * t8;
  t152 = t5 * t151;
  t153 = 0.2e1 * t152;
  t154 = param.d7 + param.sp7y + param.sp7z;
  t155 = param.d7 - param.sp7y + param.sp7z;
  t157 = param.m7 * t155 * t154;
  t158 = param.sp6y ^ 2;
  t159 = t110 - t158;
  t160 = param.m6 * t159;
  t161 = t150 - t153 - t157 + t160 - param.I6xx + param.I6yy - param.I7yy + param.I7zz;
  t162 = t35 ^ 2;
  t163 = t162 * t161;
  t164 = -t56;
  t165 = t5 * t164;
  t166 = t51 * t8;
  t167 = param.sp6y * t10;
  t168 = t165 + t166 + t167 - param.I6xy;
  t169 = t41 * t168;
  t170 = t35 * t169;
  t171 = 0.2e1 * t170;
  t172 = param.sp7z + param.d7 + param.sp7x;
  t173 = param.sp7z + param.d7 - param.sp7x;
  t175 = param.m7 * t173 * t172;
  t176 = t158 - t111;
  t177 = param.m6 * t176;
  t178 = (param.sp5x ^ 2);
  t179 = (param.sp5y ^ 2);
  t180 = t178 - t179;
  t181 = param.m5 * t180;
  t182 = t163 - t171 + t150 - t153 + t175 + t177 + t181 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t185 = t35 * t30;
  t186 = t8 * t164;
  t187 = t186 - t58 + param.I6yz;
  t188 = t187 * t41;
  t190 = param.sp4x * param.sp4z * param.m4;
  t191 = t148 * (t35 * (t130 + t131 - t132 - t133 + t134 + t135) + 0.2e1 * t53 + t144 + t145 - t146) + t67 * t16 * t182 + t185 - t53 + t188 - t190 - t62 + param.I4xz + param.I5xy;
  t192 = cos(t1);
  t193 = t192 * t191;
  t194 = -t51;
  t195 = t5 * t194;
  t196 = t195 + t186 - t58 + param.I6yz;
  t197 = t35 * t196;
  t199 = t20 * t128 * t41;
  t200 = 0.2e1 * t199;
  t202 = t70 * t7;
  t203 = t27 * t41 - t202;
  t204 = t5 * t203;
  t205 = -t29 - t17 + param.I6xz + param.I7xy;
  t206 = t205 * t41;
  t207 = t8 * t70;
  t208 = t207 * t3;
  t210 = param.m6 * t70 * param.sp6z;
  t212 = param.d5 * param.sp5y * param.m5;
  t214 = param.sp5y * param.sp5z * param.m5;
  t215 = t197 + t200 + t204 + t206 - t208 - t210 + t212 + t214 - param.I5yz;
  t218 = t168 * t16;
  t219 = t162 * t2;
  t221 = 0.2e1 * t219 * t218;
  t223 = t20 * t41 * t26;
  t224 = t41 * t151;
  t226 = t70 * t3;
  t227 = -0.2e1 * t224 - t226;
  t230 = t41 * (-t157 + t160 + param.I6yy - param.I7yy + param.I7zz - param.I6xx);
  t231 = t9 + t10;
  t232 = t70 * t231;
  t233 = t227 * t5 + t223 + t230 + t232;
  t234 = t233 * t16;
  t235 = t35 * t2;
  t236 = t235 * t234;
  t237 = t16 * t56;
  t238 = t5 * t237;
  t240 = param.m7 * t48 + t39;
  t241 = t240 * t70;
  t242 = t41 * t241;
  t243 = t194 * t8;
  t245 = param.d5 * param.sp5x * param.m5;
  t247 = param.sp5x * param.sp5z * param.m5;
  t249 = t16 * (-t242 + t243 + t245 + t247 - t167 - param.I5xz + param.I6xy);
  t250 = param.m4 * param.sp4y;
  t251 = param.sp4z * t250;
  t255 = param.sp3x * param.sp3y * param.m3;
  t258 = cos(t126);
  t260 = 0.2e1 * t258 * (t193 + t67 * t2 * t215 + t221 + t236 + t2 * (t238 + t249 + t251 - param.I4yz) + t255 - param.I3xy) * t127;
  t263 = 0.4e1 * t51;
  t264 = t8 * t263;
  t265 = 0.4e1 * t167;
  t266 = 0.4e1 * param.I6xy;
  t268 = t162 * (0.4e1 * t5 * t164 + t264 + t265 - t266);
  t274 = 0.2e1 * t157;
  t275 = 0.2e1 * t159;
  t276 = param.m6 * t275;
  t277 = 0.2e1 * param.I6xx;
  t278 = 0.2e1 * param.I6yy;
  t279 = 0.2e1 * param.I7yy;
  t280 = 0.2e1 * param.I7zz;
  t286 = t5 * t139;
  t287 = 0.2e1 * t242;
  t288 = 0.2e1 * t194;
  t289 = t8 * t288;
  t290 = 2 * t245;
  t291 = 2 * t247;
  t292 = 0.2e1 * t167;
  t293 = 0.2e1 * param.I5xz;
  t294 = 0.2e1 * param.I6xy;
  t297 = t5 * t51;
  t298 = t297 + t57 + t58 - param.I6yz;
  t299 = t16 * t298;
  t300 = t35 * t299;
  t301 = 0.2e1 * t300;
  t302 = t16 * t41;
  t304 = t20 * t128 * t302;
  t305 = 0.4e1 * t304;
  t306 = -t203;
  t307 = t16 * t306;
  t308 = t5 * t307;
  t309 = 0.2e1 * t308;
  t310 = -t205;
  t313 = 0.2e1 * t208;
  t314 = 0.2e1 * t210;
  t315 = 2 * t212;
  t316 = 2 * t214;
  t317 = 0.2e1 * param.I5yz;
  t319 = t16 * (0.2e1 * t310 * t41 + t313 + t314 - t315 - t316 + t317);
  t321 = (param.sp4x * param.sp4y * param.m4);
  t322 = 2 * t321;
  t323 = 2 * param.I4xy;
  t325 = t192 ^ 2;
  t328 = t148 * t182 * t2;
  t331 = t13 * t11;
  t333 = t35 * (-0.2e1 * t2 * t31 + t331);
  t335 = 0.2e1 * t2 * t64;
  t338 = t2 * t161;
  t339 = t162 * t338;
  t341 = t5 * t56 * t41;
  t345 = (-t341 + t41 * (t166 + t167 - param.I6xy) + t241) * t2;
  t347 = 0.2e1 * t35 * t345;
  t348 = 0.2e1 * t24;
  t350 = 0.2e1 * param.I7xx;
  t352 = t20 * (param.m7 * t348 + t279 - t350);
  t353 = t41 * t70;
  t354 = t353 * t3;
  t355 = 0.2e1 * t354;
  t357 = -t355 - 0.4e1 * t151;
  t358 = t5 * t357;
  t359 = t41 * t232;
  t360 = 0.2e1 * t359;
  t364 = t23 + (param.d5 + param.d6 + param.sp7x) * (param.d5 + param.d6 - param.sp7x);
  t365 = param.m7 * t364;
  t366 = t81 + t83 + t84 + t110 - t111;
  t367 = param.m6 * t366;
  t368 = param.m5 * t81;
  t369 = param.d5 * param.m5;
  t371 = 2 * param.sp5z * t369;
  t372 = param.sp5z ^ 2;
  t373 = -t179 + t372;
  t374 = param.m5 * t373;
  t375 = (-t75 + t76);
  t376 = (param.m4 * t375);
  t377 = t352 + t358 + t360 + t365 + t367 + t368 + t371 + t374 - param.I7yy + param.I7xx + t376 + param.I4xx - param.I4yy + param.I5yy - param.I5zz - param.I6xx + param.I6zz;
  t378 = t2 * t377;
  t382 = t162 * (t286 + t289 - t292 + t294);
  t386 = -param.m6 * t159;
  t388 = t41 * (t157 + t386 + param.I6xx - param.I6yy + param.I7yy - param.I7zz);
  t389 = -t227 * t5 - t223 - t232 + t388;
  t392 = t67 * (t35 * t389 + param.I5xz - param.I6xy + t165 + t166 + t167 + t242 - t245 - t247 + t382);
  t393 = t240 * t13;
  t394 = t2 * t393;
  t395 = t394 - t299;
  t396 = t35 * t395;
  t398 = t5 * param.m7 * t69;
  t399 = t41 * t231;
  t400 = param.m6 * t70;
  t401 = param.m5 * param.sp5z;
  t403 = (-t398 + t399 + t71 + t400 + t369 + t250 + t401) * t13;
  t404 = t2 * t403;
  t405 = 0.2e1 * t304;
  t407 = t16 * (t206 - t208 - t210 + t212 + t214 - param.I5yz);
  t409 = param.d3 * param.sp3x * param.m3;
  t411 = param.m3 * param.sp3x * param.sp3z;
  t412 = t325 * (t67 * (t268 + t35 * (0.2e1 * t223 + t5 * (-0.4e1 * t224 - 0.2e1 * t226) + t41 * (-t274 + t276 - t277 + t278 - t279 + t280) + 0.2e1 * t232) + t286 - t287 + t289 + t290 + t291 - t292 - t293 + t294) + t301 - t305 + t309 + t319 - t322 + t323) + t192 * (-t328 + t67 * (t333 + t335 + t45) - t339 + t347 + t378 - t123) + t392 + t396 + t404 + t405 - t308 + t407 - t409 - t411 + t321 + param.I3xz - param.I4xy;
  t414 = t67 * t215;
  t416 = 0.2e1 * t162 * t218;
  t417 = t35 * t234;
  t422 = -t20 * t19;
  t423 = t422 + t28 - t29 - t17 + param.I6xz + param.I7xy;
  t424 = t35 * t423;
  t425 = t424 + t53 + t60 + t62 - param.I5xy;
  t427 = t148 * t2;
  t429 = 0.2e1 * t427 * t425 * t127;
  t430 = t162 * t16;
  t431 = t430 * t338;
  t432 = t16 * t2;
  t435 = 0.2e1 * t35 * t432 * t169;
  t436 = t150 - t153 + t175 + t177 + t181 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t438 = t2 * t436 * t16;
  t440 = -t5 * t7 - t116 + t117 - t118;
  t441 = t13 * t440;
  t446 = t16 * t331;
  t447 = t2 * t423 + t446;
  t450 = t53 + t60 + t190 + t62 - param.I4xz - param.I5xy;
  t453 = t16 * t127;
  t455 = param.m3 * param.sp3y;
  t457 = -param.m4 * param.sp4z + t455;
  t458 = param.d3 * t457;
  t459 = param.sp4z * t95;
  t460 = param.sp3z * t455;
  t466 = q(2);
  t467 = sin(t466);
  t469 = cos(t466);
  t472 = t67 * (t35 * (-0.2e1 * t14 + 0.2e1 * t31) - t47 - 0.2e1 * t64) + param.I1zz - t74 + t78 + t80 + param.m7 * (t23 + t22 + t81 + t83 + t84 + t85) + (t88 + t89) * param.m1 + t94 + t99 + (param.m3 * (t100 + t101)) + param.m2 * (t104 + t105) + param.m6 * (t93 + t109 + t79 + t81 + t83 + t84 + t110 + t111) + t125 + t260 - 0.2e1 * t469 * t467 * (t258 * t412 - t192 * t127 * (t414 + t416 + t417 + t238 + t249 + t251 - param.I4yz) + t429 - t67 * (-t431 + t435 - t438 + t441) * t127 - t35 * t127 * t447 - t2 * t127 * t450 - t453 * t45 + t127 * (t458 - t459 + t460 - param.I3yz) + param.sp2y * param.sp2x * param.m2 - param.I2xy);
  t473 = t35 * t240;
  t481 = -t8 * t288;
  t482 = -t5 * t139 + t292 - t294 + t481;
  t485 = t5 * t56;
  t486 = t162 * t482 + t233 * t35 - param.I5xz + param.I6xy - t167 - t242 + t243 + t245 + t247 + t485;
  t487 = t67 * t486;
  t488 = t310 * t41;
  t490 = t16 * (t488 + t208 + t210 - t212 - t214 + param.I5yz);
  t494 = 0.2e1 * t192 * t2 * (t487 + t300 - t405 + t308 + t490 - t321 + param.I4xy);
  t497 = 0.2e1 * t67 * t425 * t16;
  t498 = param.sp3y ^ 2;
  t499 = -t100 + t498;
  t501 = param.sp4z ^ 2;
  t502 = -t76 + t501;
  t504 = -t81 - t83 - t84 - t110 + t158;
  t509 = -t23 - (param.sp7z + param.d5 + param.d6 + param.d7) * (-param.sp7z + param.d5 + param.d6 - param.d7);
  t511 = t178 - t372;
  t513 = (param.m3 * t499) + (param.m4 * t502) + (param.m5 * t511) + param.m6 * t504 + param.m7 * t509 + param.I3xx + param.I4yy + param.I5zz + param.I7yy - t360 - t371 - t494 + t497;
  t516 = -param.m7 * t24 + param.I7xx - param.I7yy;
  t517 = t20 * t516;
  t518 = 0.4e1 * t152;
  t519 = t352 - t518 - t274 + t276 - t277 + t278 - t279 + t280;
  t521 = t517 + t153 + t157 + t386 + param.I6xx - param.I6yy + param.I7yy - param.I7zz;
  t522 = t162 * t521;
  t523 = -t176;
  t524 = param.m6 * t523;
  t525 = -t180;
  t526 = param.m5 * t525;
  t527 = t522 + t171 + t517 + t153 - t175 + t524 + t526 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t528 = t148 * t527;
  t529 = 0.2e1 * t341;
  t531 = t41 * (t481 + t292 - t294);
  t532 = 0.2e1 * t241;
  t534 = t35 * (-t529 + t531 + t532);
  t536 = t371 + t374 - param.I7yy + param.I7xx + t376 + param.I4xx - param.I4yy + param.I5yy - param.I5zz - param.I6xx + param.I6zz;
  t539 = 0.4e1 * t341;
  t547 = 0.2e1 * t354 + 0.2e1 * t151;
  t548 = t5 * t547;
  t549 = t517 + param.I6xx - param.I6yy - param.I7zz - t368 + t162 * t519 + t325 * (t528 + t497 + t522 + t534 + t352 + t358 + t360 + t365 + t367 + t368 + t536) + t528 + t35 * (t539 + t41 * (-t8 * t263 - t265 + t266) - t532) + t548 - param.I5xx - param.I3yy - param.I4zz;
  t551 = t258 ^ 2;
  t556 = t35 * (-t539 + t41 * (t264 + t265 - t266) + t532);
  t558 = -t5 * t547;
  t560 = -param.m5 * t511;
  t562 = -param.m4 * t502;
  t564 = -param.m3 * t499;
  t565 = -param.I5zz - param.I4yy - param.I7yy - param.I3xx + t494 - t497 + t360 + t371 + t556 + t558 + t560 + t562 + t564;
  t567 = -param.m7 * t509;
  t569 = -param.m6 * t504;
  t570 = t148 * t182;
  t572 = t41 * (t289 - t292 + t294);
  t574 = t35 * (t529 + t572 - t532);
  t578 = t20 * (-param.m7 * t348 - t279 + t350);
  t580 = -t5 * t357;
  t582 = -param.m7 * t364;
  t584 = -param.m6 * t366;
  t587 = -param.m5 * t373;
  t589 = -param.m4 * t375;
  t590 = -t371 + t587 + param.I7yy - param.I7xx + t589 - param.I4xx + param.I4yy - param.I5yy + param.I5zz + param.I6xx - param.I6zz;
  t592 = t325 * (t570 - t497 + t163 + t574 + t578 + t580 - t360 + t582 + t584 - t368 + t590);
  t596 = t162 * (-param.m6 * t275 + t274 + t277 - t278 + t279 - t280 + t518 + t578);
  t597 = t567 + t569 + t592 + t596 + t570 - param.I6xx + param.I6yy + param.I7zz + t368 + t150 + param.I5xx + param.I3yy + param.I4zz;
  t609 = (2 * param.m5 * t525) + 0.2e1 * param.m6 * t523 + 0.2e1 * param.I5xx - 0.2e1 * param.I5yy - 0.2e1 * param.I6zz + 0.4e1 * t170 - 0.2e1 * t175 + t278 + t280 - t350 + t518 + t578 + t596;
  t613 = 0.2e1 * t67 * t2 * t486;
  t641 = param.I4xx + param.I7xx - t80 - t125 - t260 - t153 - t171 + t551 * (t565 + t597) + t592 + t148 * t609 + t192 * (t613 + t35 * (0.2e1 * t2 * t299 - 0.2e1 * t393) + t2 * (-t305 + t309 + t319 - t322 + t323) - 0.2e1 * t403) + t67 * (t35 * (0.2e1 * t14 - 0.4e1 * t31) + t47 + 0.4e1 * t64) + param.m7 * (-t22 - (param.d3 + param.sp7z + param.d4 + param.d7) * (param.d3 - param.sp7z + param.d4 - param.d7)) + (param.m4 * (-t75 + t501)) + (param.m3 * (t498 - t101)) + param.m5 * (-t93 - t109 - t79 + t178 - t179);
  t648 = param.sp2x ^ 2;
  t651 = -t93 * t92 - (param.d3 * t98) + param.m6 * (-t93 - t109 - t79 + t158 - t111) + param.m2 * (t648 - t104) - param.I6yy - param.I7zz + t163 + t150 + param.I2yy + param.I3zz + param.I5yy + param.I6zz - param.I5xx - param.I3yy - param.I4zz - param.I2xx;
  t653 = t469 ^ 2;
  t657 = 0.2e1 * t192 * (t473 - t398 + t399 + t71 + t400 + t369 + t401 + t250) * t13 + t360 + t371 + t551 * (t513 + t549) + t534 + t522 + t653 * (t641 + t651) + t570 + param.I6yy + param.I7zz + t368 + param.I5xx + param.I3yy + param.I4zz + param.I2xx + param.m5 * (t93 + t109 + t79 + t179 + t372);
  m(1,1) = t472 + t657;
  t658 = t35 * t298;
  t659 = t5 * t306;
  t662 = t67 * t2 * (t658 - t200 + t659 + t488 + t208 + t210 - t212 - t214 + param.I5yz);
  t663 = t485 - t242 + t243 + t245 + t247 - t167 - param.I5xz + param.I6xy;
  t664 = t16 * t663;
  t671 = t185 - t53 + t188 - t62 + param.I5xy;
  t674 = 0.2e1 * t67 * t671 * t16;
  t676 = t528 - t674 + t522 + t534 + t352 + t358 + t360 + t365 + t367 + t368 + t536;
  t678 = t325 * t127;
  t680 = -t200 + t659 + t488 + t208 + t210 - t212 - t214 + param.I5yz;
  t682 = t16 * t680 + param.I4xy + t300 - t321 + t487;
  t691 = t570 + t674 + t596 + t556 + t150 + t558 + t360 + t567 + t569 + t368 + t371 + t560;
  t692 = -param.I7yy + t562 + param.I6yy + param.I7zz - param.I6xx + t564 - param.I3xx + param.I3yy - param.I4yy + param.I4zz + param.I5xx - param.I5zz;
  t693 = t691 + t692;
  t699 = -t440;
  t700 = t13 * t699;
  t705 = t41 * t240;
  t707 = (t705 - t43) * t13;
  t708 = t16 * t707;
  t722 = (t16 * t699 - t121) * t13;
  t734 = t242 + t165 + t166 - t245 - t247 + t167 + param.I5xz - param.I6xy;
  t735 = t734 * t16;
  m(1,2) = 0.2e1 * t551 * (t193 - t662 + t221 + t236 + t2 * (t664 + t251 - param.I4yz) + t255 - param.I3xy) * t467 + t258 * (-t678 * t676 * t467 + t192 * (0.2e1 * t127 * t467 * t2 * t682 + (t414 + t416 + t417 + t664 + t251 - param.I4yz) * t469) + t127 * t693 * t467 + (0.2e1 * t148 * t2 * t671 + t67 * (-t431 + t435 - t438 - t700) + t35 * t447 + t2 * t450 - t708 - (param.d3 * t457) + t459 - t460 + param.I3yz) * t469) + 0.2e1 * t678 * t469 * t682 + t192 * (t127 * t469 * (-t328 + t67 * (t333 + t335 - t707) - t339 + t347 + t378 + t722) - t191 * t467) + t127 * (t392 + t396 + t404 + t16 * (t200 + t204 + t206 - t208 - t210 + t212 + t214 - param.I5yz) - t409 - t411 + t321 + param.I3xz - param.I4xy) * t469 + t467 * (t662 - t221 - t236 + t2 * (t735 - t251 + param.I4yz) - param.sp2z * param.sp2x * param.m2 - t255 + param.I2xz + param.I3xy) + (-param.m2 * param.sp2y * param.sp2z + param.I2yz) * t469;
  t746 = t392 - t300 + t405 - t308 + t407 + t321 - param.I4xy;
  t750 = t485 + t243 - t167 + param.I6xy;
  t751 = t41 * t750;
  t753 = 0.2e1 * t35 * t751;
  t754 = t522 - t753 + t517 + t153 - t175 + t524 + t526 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t760 = -t11;
  t774 = t750 * t16;
  t776 = 0.2e1 * t127 * t774;
  t784 = t578 + t580 - t360 + t582 + t584 - t368 - t371 + t587 + param.I7yy - param.I7xx + t589 - param.I4xx + param.I4yy - param.I5yy + param.I5zz + param.I6xx - param.I6zz;
  t787 = -t119 * t16;
  t804 = t2 * t750;
  t805 = t41 * t804;
  t818 = t760 * t16;
  t839 = t163 + t753 + t150 - t153 + t175 + t177 + t181 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t841 = t35 * t241;
  t842 = 0.2e1 * t841;
  t843 = param.sp7z ^ 2;
  t845 = 0.2e1 * param.d7 * param.sp7z;
  t846 = param.d7 ^ 2;
  t847 = t70 ^ 2;
  t848 = -t23 - t843 - t845 - t846 - t847;
  t850 = -t81 - t83 - t84 - t110 - t158;
  t852 = -t178 - t372;
  t854 = -t76 - t501;
  t856 = -t100 - t498;
  t858 = t325 * (t148 * t754 + t352 + t358 + t360 + t365 + t367 + t368 + t497 + t522 + t534 + t536) + 0.2e1 * t192 * t2 * t746 + t148 * t839 - t497 - t842 + t517 + t548 - t360 + param.m7 * t848 + param.m6 * t850 - t368 - t371 + (param.m5 * t852) - param.I7xx + (param.m4 * t854) - param.I5yy - param.I6zz + (param.m3 * t856) - param.I3zz - param.I4xx;
  m(1,3) = t467 * (0.2e1 * t325 * t258 * t746 + t192 * (-t427 * t258 * t754 + t67 * (t35 * (t258 * (-0.2e1 * t16 * t2 * t423 + t13 * t760) - t127 * t298) + t258 * (-t335 + t707) - t127 * t680) + t162 * (-t2 * t258 * t521 - t776) + t35 * (-0.2e1 * t258 * t345 - t389 * t453) + t258 * (t2 * t784 - (t115 + t787 - t121) * t13) - t127 * (-t238 + t16 * (t242 + t166 - t245 - t247 + t167 + param.I5xz - param.I6xy) - t251 + param.I4yz)) - t429 + t67 * (t162 * (t127 * t432 * t521 + t258 * t482) + t35 * (t233 * t258 - 0.2e1 * t453 * t805) + t258 * t663 - (t438 + t700) * t127) + t35 * (-t258 * t395 - t127 * (t13 * t818 + t2 * t30)) + t258 * (-t404 - t405 + t308 + t490 + t409 + t411 - t321 - param.I3xz + param.I4xy) - (t2 * (-t53 + t188 - t190 - t62 + param.I4xz + param.I5xy) + t708 + t458 - t459 + t460 - param.I3yz) * t127) - t469 * t858;
  t860 = t18 * t8;
  t862 = 0.2e1 * t5 * t860;
  t865 = -param.m7 * t155 * t154;
  t866 = t517 - t862 - t865 + t386 + param.I6xx - param.I6yy + param.I7yy - param.I7zz;
  t867 = t866 * t127;
  t869 = t8 * t516;
  t870 = t5 * t869;
  t871 = t422 - t870 - t29 - t17 + param.I6xz + param.I7xy;
  t872 = t871 * t258;
  t879 = t41 * t194;
  t880 = t5 * t879;
  t881 = t880 + t188 - t62 + param.I5xy;
  t887 = -param.m7 * t173 * t172;
  t888 = t517 - t862 + t887 + t524 + t526 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  t896 = t192 * t258;
  t917 = 0.2e1 * t20 * t18 * t41;
  t919 = t516 * t41 * t8;
  t920 = t919 + t202;
  t922 = t5 * t920 + param.I5yz + t208 + t210 - t212 - t214 + t488 + t917;
  t928 = t16 * t258;
  t934 = t20 * t41 * t516;
  t937 = -0.2e1 * t41 * t860 + t226;
  t938 = t5 * t937;
  t941 = t934 + t938 + t41 * (-t865 + t386 + param.I6xx - param.I6yy + param.I7yy - param.I7zz) - t232;
  t942 = t2 * t941;
  t948 = t5 * t164 * t41;
  t962 = t880 + t188 - t190 - t62 + param.I4xz + param.I5xy;
  t968 = -t251 + param.I4yz;
  t972 = t23 + t22 + t847;
  t973 = param.m7 * t972;
  t974 = t81 + t83 + t84 + t110 + t111;
  t975 = param.m6 * t974;
  t976 = t179 + t372;
  t977 = t976 * param.m5;
  t978 = -t74 + t360 + t973 + t975 + t368 + t371 + t977 + param.I6yy + param.I7zz + t78 + param.I4zz + param.I5xx;
  t982 = t21 + t870 + t29 + t17 - param.I6xz - param.I7xy;
  t983 = t35 * t982;
  t984 = t983 + t880 + t188 - t62 + param.I5xy;
  t987 = 0.2e1 * t148 * t2 * t984;
  t1000 = t192 * t750;
  t1005 = t2 * t871;
  t1008 = t734 * t192;
  t1010 = t968 * t192;
  m(1,4) = t467 * (t148 * (t162 * t867 + t35 * (-0.2e1 * t127 * t41 * t750 - 0.2e1 * t192 * t872) + 0.2e1 * t192 * t258 * t881 + t888 * t127) + t67 * (t162 * t258 * t866 * t16 * t192 + t35 * (t16 * (0.2e1 * t127 * t871 - 0.2e1 * t751 * t896) + t2 * (t127 * t331 - t196 * t258)) + t16 * (t192 * t258 * t888 - 0.2e1 * t127 * t881) + t2 * (t127 * t45 + t258 * t922)) + t162 * (0.2e1 * t804 * t928 - t867) + t35 * (t928 * t942 + t192 * (-t127 * t393 + t872) + 0.2e1 * (-t948 + t41 * (t243 - t167 + param.I6xy) - t241) * t127) - t16 * t2 * (t127 * t441 - t258 * t734) + t192 * (-t127 * t403 - t258 * t962) + t2 * (-t121 * t127 * t13 + t258 * t968) - t978 * t127) + t469 * (-t987 + t67 * (-t219 * t866 * t16 + t35 * (-t192 * t196 + 0.2e1 * t432 * t751) - t16 * t2 * t888 + t192 * t922) + 0.2e1 * t430 * t1000 + t35 * (t16 * t192 * t941 - t1005) + t16 * t1008 + t1010 + t962 * t2);
  t1017 = t2 * t866;
  t1022 = t70 * t11;
  t1023 = t41 * t866 - t1022;
  t1030 = t196 * t16;
  t1032 = 0.2e1 * t805;
  t1035 = t192 * t331;
  t1036 = t150 + t862 + t865 + t160 + param.I6yy - param.I7yy + param.I7zz - param.I6xx;
  t1037 = t41 * t1036;
  t1045 = t41 * t871;
  t1046 = t5 * t70;
  t1047 = t1046 * t7;
  t1054 = t16 * (t41 * t982 + param.I5yz + t1047 + t208 + t210 - t212 - t214);
  t1056 = t16 * t441;
  t1057 = t48 ^ 2;
  t1058 = t22 + t1057;
  t1059 = param.m7 * t1058;
  t1060 = t158 + t111;
  t1061 = param.m6 * t1060;
  t1062 = t178 + t179;
  t1063 = t1062 * param.m5;
  t1065 = (t517 - t862 + t1059 + t1061 + t1063 + param.I5zz + param.I6xx + param.I7yy) * t2;
  t1076 = t866 * t192;
  t1086 = t2 * t734;
  m(1,5) = t467 * (t162 * (0.2e1 * t1000 * t258 * t67 - t1017 * t258 - t776) + t35 * (t67 * (t258 * (t1023 * t192 - t331) + t127 * t196) + t258 * (t1030 * t192 + t1032) + (t1035 + t1037 + t1022) * t453) + t67 * (t258 * (t1008 - t45) + t127 * (t192 * t441 - param.I5yz + t1045 - t1047 - t208 - t210 + t212 + t214)) + t258 * (-t1054 * t192 + t1056 + t1065) + t127 * t16 * (t192 * t45 - param.I5xz + param.I6xy - t167 - t242 + t243 + t245 + t247 + t485)) - (t162 * (0.2e1 * t67 * t804 + t1076) + t35 * (t1023 * t2 * t67 + t1030 * t2 - 0.2e1 * t192 * t751) + t67 * t1086 + t192 * (-(t1062 * param.m5) - param.m6 * t1060 - param.m7 * t1058 - param.I5zz - param.I6xx - param.I7yy + t150 + t862) - t2 * t1054) * t469;
  t1106 = t192 * t70 + param.d3 + param.d4;
  t1119 = t194 * t2;
  t1122 = t127 * param.sp7x;
  t1133 = t67 * t13;
  t1135 = t192 * t240 * t1133;
  t1136 = t67 * t241;
  t1144 = t258 * t1106 * t3;
  t1145 = t194 * t127;
  t1153 = t23 + t1057;
  t1154 = param.m7 * t1153;
  t1155 = t110 + t158;
  t1156 = t1155 * param.m6;
  t1157 = t150 + t862 + t1154 + t1156 + param.I6zz + param.I7xx;
  t1161 = t2 * t258;
  t1162 = t20 * t18;
  t1172 = param.m7 * t127;
  t1173 = t192 * t13;
  t1174 = t1173 + param.d5 + param.d6;
  t1196 = t16 * t70;
  t1198 = t2 * t67;
  t1201 = t192 * t194;
  t1210 = t1022 * t41 + param.I6zz + param.I7xx + t1154 + t1156 + t150 + t862;
  m(1,6) = t467 * (t35 * (t16 * (t127 * t5 * t516 * t8 + t1106 * t240 * t258 + 0.2e1 * t127 * t18 * t20 - t127 * t205) - 0.2e1 * t20 * t18 * t67 * t896 + t5 * (t258 * (-t192 * t516 * t67 * t8 + t1119) + t13 * param.m7 * t2 * t1122) + t258 * (t192 * t205 * t67 + t187 * t2) - t127 * (t13 * t2 * t231 - t1135 - t1136)) + t16 * (t41 * (t5 * (-t1144 + t1145) + t258 * t1106 * t231 + t127 * t187) + t1157 * t896) + t41 * (-0.2e1 * t1162 * t1161 + t5 * (t258 * (-t192 * t194 * t67 - t2 * t516 * t8) - t1174 * t1172 * param.sp7x * t67) + t258 * (-t187 * t192 * t67 + t2 * t205) + t127 * (t1173 * t231 * t67 + t232 * t67 + t394)) + t1157 * t127 * t67) + t469 * (t35 * (-t1196 * t240 * t2 + 0.2e1 * t1162 * t1198 + t5 * (t1198 * t869 + t1201) + t192 * t187 - t205 * t1198) - t16 * t1210 * t2 + (-0.2e1 * t20 * t18 * t192 + t5 * (t1198 * t194 - t192 * t869) + t192 * t205 + t187 * t1198) * t41);
  t1230 = -t1174 * t127 * t7 - t164 * t192 * t258;
  t1236 = t258 * param.sp7y * t1106 * param.m7;
  t1237 = t164 * t127;
  t1244 = -param.sp7x * t1172 * t1174 + t1201 * t258;
  t1248 = t13 * param.m7;
  t1263 = t2 * t41;
  t1271 = t67 * t192;
  t1279 = param.m7 * (t22 + t23) + param.I7zz;
  t1284 = t164 * t35 - t226;
  t1287 = t2 * param.sp7y;
  t1289 = t35 * t71;
  t1292 = t41 * t192;
  t1296 = t35 * t194;
  t1300 = t2 * param.sp7x;
  t1308 = t41 * t67;
  m(1,7) = t467 * (t5 * (t67 * (t1230 * t35 + t1144 + t1145) - t35 * t16 * (t1236 - t1237) + t1244 * t16 - t2 * t41 * (param.sp7y * t1248 * t127 + t164 * t258)) + t8 * (t67 * (t1244 * t35 - t1236 + t1237) - t35 * t16 * (t1144 + t1145) - t16 * t1230 - (t1122 * t1248 + t258 * t51) * t1263) - t1279 * (-t127 * t16 * t41 + t1271 * t258 * t41 - t1161 * t35)) - t469 * (t5 * (-t1284 * t2 * t67 - t1287 * t1289 * t16 + t1119 * t16 + t1292 * t164) + t8 * (t67 * t2 * (t1296 - t202) - t1289 * t16 * t1300 + t16 * t164 * t2 - t41 * t1201) - t1279 * (t1308 * t2 + t192 * t35));
  m(2,1) = m(1,2);
  t1315 = param.I5zz + param.I4yy + param.I7yy + param.I3xx + t80 + t94 + t99 - t171 + t517 + param.I2zz + param.I6xx + t163;
  t1316 = t5 * t2;
  t1342 = t13 * t16;
  t1350 = t41 * t310 * t16;
  t1351 = t208 + t210 - t212 - t214 + param.I5yz;
  t1370 = t13 * t41;
  t1376 = -t1351;
  t1398 = -0.2e1 * t258 * (-t662 + t221 + t236 + t1316 * t237 + t2 * (-t353 * t240 * t16 + t16 * (t243 - t167 + t245 + t247 - param.I5xz + param.I6xy) + t251 - param.I4yz) + t255 - param.I3xy) * t127 - 0.2e1 * t2 * (t787 - t121) * t13 - t325 * t676 * (0.1e1 + t258) * (t258 - 0.1e1) - 0.2e1 * t1198 * (t11 * t35 + t42 + t43) * t13 + t5 * (-0.2e1 * t1342 * t2 * t7 + 0.2e1 * t151) + t551 * t693 + t192 * (0.2e1 * t551 * t2 * (t1351 * t16 + param.I4xy + t1350 + t300 + t308 - t321 - t405 + t487) - 0.2e1 * t258 * t127 * t191 - t613 + t35 * (-0.2e1 * t1316 * t16 * t51 - 0.2e1 * t16 * t2 * t59 + 0.2e1 * t393) + 0.4e1 * t199 * t432 + t5 * (-0.2e1 * t1370 * t3 - 0.2e1 * t2 * t307) + t2 * (0.2e1 * t1376 * t16 - 0.2e1 * t1350 + t322 - t323) + 0.2e1 * (t399 + t71 + t400 + t369 + t250 + t401) * t13) + param.m5 * (t93 + t109 + t79 + t178 + t179) + (param.m4 * (t75 + t501)) + (param.m3 * (t498 + t101)) + (t648 + t104) * param.m2 + param.m6 * (t93 + t109 + t79 + t158 + t111) + param.m7 * (t22 + t843 + t845 + t846 + t85);
  m(2,2) = t1315 + t1398;
  t1400 = t760 * t70;
  t1418 = t298 * t41 - param.I5xy + t424 + t62;
  t1432 = t522 + t35 * (t41 * t482 + t532) - 0.2e1 * t41 * t1400 + t352 - t518 + t365 + t367 + t368 + t371 + t374 - param.I7yy + param.I7xx + t376 + param.I4xx - param.I4yy + param.I5yy - param.I5zz - param.I6xx + param.I6zz;
  t1437 = t41 * t521 + t1400;
  t1445 = t41 * t423;
  t1448 = t325 * (t67 * (t268 + t35 * (t41 * t519 - 0.2e1 * t1400) - t287 + t286 + t289 + t290 + t291 - t292 - t293 + t294) + t301 + t16 * (t41 * (-t20 * t129 - t131 + t132 + t133 - t134 - t135) + 0.2e1 * t1047 + t313 + t314 - t315 - t316 + t317) - t322 + t323) + t192 * (t148 * t2 * t754 + t67 * (0.2e1 * t2 * t1418 * t16 - (t35 * t760 - t43 + t705) * t13) + t2 * t1432 + t722) + t67 * (t1437 * t35 + param.I5xz - param.I6xy + t165 + t166 + t167 + t242 - t245 - t247 + t382) + t2 * (t11 * t41 + t250 + t369 + t400 + t401 + t473 + t71) * t13 - t300 + t16 * (t1445 - t1047 - t208 - t210 + t212 + t214 - param.I5yz) - t409 - t411 + t321 + param.I3xz - param.I4xy;
  t1454 = 0.2e1 * t162 * t774;
  t1466 = t41 * t196;
  m(2,3) = t127 * t1448 - (t192 * (t67 * (t30 * t41 + param.I5yz + t1047 + t208 + t210 - t212 - t214 + t658) + t1454 + t35 * t1437 * t16 + t735 - t251 + param.I4yz) + 0.2e1 * t148 * t2 * t1418 + t67 * (-t16 * t2 * t754 + t700) + t2 * (t185 + t1466 - t190 - t62 + param.I4xz + param.I5xy) + t35 * t13 * t818 + t708 + t458 - t459 + t460 - param.I3yz) * t258;
  t1474 = t162 * t866 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz + t517 + t524 + t526 - t753 - t862 + t887;
  t1488 = 0.2e1 * t948;
  t1504 = t148 * t1474 + t67 * (t35 * (t16 * (t130 - 0.2e1 * t870 - t132 - t133 + t134 + t135) + t14) + t16 * (-0.2e1 * t880 + t144 + t145 - t146) + t46) + t162 * t1036 + t35 * (-t192 * t393 - t1488 - t532 + t572) - t1342 * t440 * t2 - t192 * t403 - t2 * t13 * t121 + t74 - t360 - param.m7 * t972 - param.m6 * t974 - t368 - t371 - (t976 * param.m5) - (t77 * param.m4) - param.I4zz - param.I5xx - param.I6yy - param.I7zz;
  t1512 = t2 * t196;
  m(2,4) = t258 * t1504 + (-0.2e1 * t148 * t984 * t192 + t67 * (-t430 * t1076 + t35 * (0.2e1 * t1292 * t774 + t1512) - t16 * t888 * t192 - t922 * t2) - 0.2e1 * t162 * t750 * t432 + t35 * (-t16 * t942 - t192 * t871) - t16 * t1086 + t192 * t962 - t2 * t968) * t127;
  t1532 = t67 * t750;
  t1544 = t67 * t663;
  m(2,5) = t127 * (t162 * (-0.2e1 * t1532 * t192 + t1017) + t35 * (t192 * (t67 * (t1037 + t1022) - t1030) + t67 * t331 - t1032) + t192 * (t1544 + t1054) + t67 * t45 - t1056 - t1065) + t258 * (-t1454 + t35 * (t11 * t1173 * t16 - t1023 * t16 + t196 * t67) + t192 * (t16 * t44 + t440 * t67) * t13 + t67 * (t1045 - t1047 - t208 - t210 + t212 + t214 - param.I5yz) - t735);
  m(2,6) = t127 * (t35 * (t192 * (-t1196 * t240 + t67 * t982) - t16 * t393 - t1512) + t192 * (-t1210 * t16 + t1466 * t67) - t41 * (t446 + t1005)) + (t35 * (t16 * t982 + t1135 + t1136 - t14) + t1035 * t1308 + t67 * t1210 + (t1030 + t394) * t41) * t258;
  t1590 = t35 * t70;
  t1592 = -t1590 * t7 + param.I7yz - t50;
  t1597 = t16 * param.sp7y;
  t1598 = t35 * t1248;
  t1607 = -t1590 * t3 + param.I7xz - t55;
  t1612 = param.sp7x * t16;
  t1638 = t35 * t67;
  t1654 = t1279 * t41;
  m(2,7) = t127 * (t5 * (t192 * (t1284 * t67 - t1592 * t16) - t1133 * t3 + t1598 * t1597 + t164 * t1263) + t8 * (t192 * (t67 * (t35 * t51 + t202) - t16 * t1607) + t1133 * t7 + t1598 * t1612 - t194 * t1263) + (t1271 * t41 - t235) * t1279) - t258 * (t5 * (t1173 * (param.sp7y * t35 * t67 + t1612) * param.m7 - t67 * t1592 - t35 * t164 * t16 + (param.sp7x * t16 * t70 + t1287 * t1370) * param.m7) + t8 * (t1173 * (param.sp7x * t1638 - t1597) * param.m7 - t67 * t1607 + t35 * t194 * t16 + param.m7 * (-param.sp7y * t16 * t70 + t1300 * t1370)) - t16 * t1654);
  m(3,1) = m(1,3);
  m(3,2) = m(2,3);
  t1659 = t162 * (t150 + t862 - t157 + t160 + param.I6yy - param.I7yy + param.I7zz - param.I6xx);
  t1660 = t1659 + t753 + t150 + t862 + t175 + t177 + t181 - param.I5xx + param.I5yy - param.I6yy + param.I6zz + param.I7xx - param.I7zz;
  t1661 = t148 * t1660;
  t1665 = 0.2e1 * t67 * t16 * (t983 - t53 + t188 - t62 + param.I5xy);
  t1674 = t382 + t35 * (t934 + t938 + t388 - t232) + t165 + t242 + t166 - t245 - t247 + t167 + param.I5xz - param.I6xy;
  t1688 = t517 - t862 + t157 + t386 + param.I6xx - param.I6yy + param.I7yy - param.I7zz;
  t1689 = t162 * t1688;
  t1690 = t1689 - t753 + t517 - t862 - t175 + t524 + t526 + param.I5xx - param.I5yy + param.I6yy - param.I6zz - param.I7xx + param.I7zz;
  m(3,3) = t325 * (t1661 + t1665 + t1659 + t574 + t578 + t5 * (t355 - 0.4e1 * t860) - t360 + t582 + t584 - t368 + t590) - 0.2e1 * t192 * t2 * (t16 * t205 * t41 - t16 * t5 * t920 + t1030 * t35 - 0.2e1 * t1162 * t302 + t1376 * t16 + t1674 * t67 - param.I4xy + t321) + t148 * t1690 - t1665 + t842 + t150 + t5 * (-0.2e1 * t354 + 0.2e1 * t860) + t360 - param.m7 * t848 - param.m6 * t850 + t368 + t371 - (param.m5 * t852) + param.I7xx - (param.m4 * t854) + param.I5yy + param.I6zz + param.I4xx - (param.m3 * t856) + param.I3zz;
  t1708 = -t5 * t920;
  m(3,4) = -t987 + t67 * (-t16 * t2 * t1690 - t192 * (t197 - t917 + t1708 + t206 - t208 - t210 + t212 + t214 - param.I5yz)) + t16 * t192 * t1674 + t2 * (t983 + t880 + t188 - t190 - t62 + param.I4xz + param.I5xy) + t1010;
  t1729 = t1659 + t753 + t517 - t862 + t1059 + t1061 + param.I7yy + param.I6xx + t1063 + param.I5zz;
  m(3,5) = t2 * (-0.2e1 * t162 * t1532 + t35 * (t67 * (-t5 * t937 + t230 + t232 - t934) - t1030) + t1544 - t16 * (-t917 + t1708 + t206 - t208 - t210 + t212 + t214 - param.I5yz)) + t1729 * t192;
  t1737 = t354 - 0.2e1 * t860;
  t1740 = t35 * t869 + t879;
  t1745 = -param.m7 * t1153;
  t1747 = -t1155 * param.m6;
  m(3,6) = t2 * (t20 * (t16 * t516 + 0.2e1 * t1638 * t18) + t5 * (t16 * t1737 + t1740 * t67) + t16 * (-t841 - t359 + t1745 + t1747 - param.I6zz - param.I7xx) - (t205 * t35 - t188) * t67) + (-t917 + t5 * (t1296 - t919) + t35 * t187 + t206) * t192;
  t1762 = t1046 * t3;
  t1763 = t207 * t7;
  t1771 = t1590 * (param.sp7x * t8 + param.sp7y * t5) * param.m7 + t297 + t57;
  t1775 = t1279 * t35;
  m(3,7) = t2 * (t67 * (t35 * (t165 - t243) - t1762 + t1763 + t1654) + t16 * t1771) + (t1775 + (t485 + t243) * t41) * t192;
  m(4,1) = m(1,4);
  m(4,2) = m(2,4);
  m(4,3) = m(3,4);
  m(4,4) = t1661 + 0.2e1 * t67 * t16 * t984 + t1689 + t35 * (t1488 + t531 + t532) - t74 + t360 + t973 + t975 + t368 + t371 + t977 + param.I6yy + param.I7zz + param.I5xx + t78 + param.I4zz;
  m(4,5) = t16 * (t382 + t35 * (t1688 * t41 - t1022) + t242 + t165 + t166 - t245 - t247 + t167 + param.I5xz - param.I6xy) - (t197 + t1045 - t1047 - t208 - t210 + t212 + t214 - param.I5yz) * t67;
  m(4,6) = t67 * (t1737 * t5 - param.I6zz - param.I7xx + t1745 + t1747 - t359 + t517 - t841) - (0.2e1 * t18 * t20 * t35 + t1740 * t5 + t310 * t35 + t188) * t16;
  t1802 = t485 - t166;
  m(4,7) = t16 * (t1802 * t35 - t1654 + t1762 - t1763) + t67 * t1771;
  m(5,1) = m(1,5);
  m(5,2) = m(2,5);
  m(5,3) = m(3,5);
  m(5,4) = m(4,5);
  m(5,5) = t1729;
  m(5,6) = t1445 - t658;
  m(5,7) = t1802 * t41 + t1775;
  m(6,1) = m(1,6);
  m(6,2) = m(2,6);
  m(6,3) = m(3,6);
  m(6,4) = m(4,6);
  m(6,5) = m(5,6);
  m(6,6) = t150 - t153 + t1154 + param.I7xx + t1156 + param.I6zz;
  m(6,7) = t195 - t57;
  m(7,1) = m(1,7);
  m(7,2) = m(2,7);
  m(7,3) = m(3,7);
  m(7,4) = m(4,7);
  m(7,5) = m(5,7);
  m(7,6) = m(6,7);
  m(7,7) = t1279;
  tmpreturn = m;
