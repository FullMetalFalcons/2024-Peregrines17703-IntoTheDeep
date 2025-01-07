package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(-12, 62,Math.toRadians(-90)))
                        .strafeToLinearHeading(new Vector2d(0, 34), Math.toRadians(-90))
                        .waitSeconds(3)
                        .strafeToLinearHeading(new Vector2d(-58, 47), Math.toRadians(-90))
                        .waitSeconds(3)
                        .turn(Math.toRadians(180))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-150))
                        .waitSeconds(2)
                        .turn(Math.toRadians(150))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-205))
                        .waitSeconds(2)
                        .turn(Math.toRadians(205))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}