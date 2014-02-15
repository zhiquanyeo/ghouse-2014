/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.ghouse.vision;

import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 *
 * @author Robotics
 */
public class RobotCamera {
    //Camera Constants for distance calculation
    final int Y_IMAGE_RES = 480;        //X Image resolution in pixels. 120, 240, 480
    final double VIEW_ANGLE = 49;       //Axis M1013
    
    //Score limits for target ID
    final int RECTANGULARITY_LIMIT = 40;
    final int ASPECT_RATIO_LIMIT = 55;
    
    //Score limits used for hot target determination
    final int TAPE_WIDTH_LIMIT = 50;
    final int VERTICAL_SCORE_LIMIT = 50;
    final int LR_SCORE_LIMIT = 50;
    
    //Minimum area of particles to be considered
    final int AREA_MINIMUM = 110;
    
    //Maximum number of particles to process
    final int MAX_PARTICLES = 8;
    
    AxisCamera camera;          //The axis camera object
    CriteriaCollection cc;      //Criteria for doing aprticle filter operation
    
    public class Scores {
        double rectangularity;
        double aspectRatioVertical;
        double aspectRatioHorizontal;
    }
    
    public class TargetReport {
        public int verticalIndex;
        public int horizontalIndex;
        public boolean Hot;
        public double totalScore;
        public double leftScore;
        public double rightScore;
        public double tapeWidthScore;
        public double verticalScore;
        public double distance;
    }
    
    //Scoring variables
    private TargetReport target;
    private int verticalTargets[];
    private int horizontalTargets[];
    private int verticalTargetCount, horizontalTargetCount;
    
    public RobotCamera() {
        camera = AxisCamera.getInstance();
        cc = new CriteriaCollection();
        cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        
        resetScoring();
    }
    
    public void resetScoring() {
        target = new TargetReport();
        verticalTargets = new int[MAX_PARTICLES];
        horizontalTargets = new int[MAX_PARTICLES];
        verticalTargetCount = horizontalTargetCount = 0;
    }
    
    //Call this every loop in autonomous/teleop, or whenever you want to get distance readings
    //This returns the BEST target
    public TargetReport getTargetReport() {
        //Actually get the image
        try {
            ColorImage image = camera.getImage();
            //Keep only green objects
//            BinaryImage thresholdImage = image.thresholdHSV(125, 154, 220, 255, 229, 255);
            BinaryImage thresholdImage = image.thresholdHSV(76, 184, 227, 255, 96, 167);
            BinaryImage filteredImage = thresholdImage.particleFilter(cc); //Filter out small objects
            
            //iterate through each particle and score to see if it is a target
            Scores scores[] = new Scores[filteredImage.getNumberParticles()];
            horizontalTargetCount = verticalTargetCount = 0;
            
            if (filteredImage.getNumberParticles() > 0) {
                for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) {
                    ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                    scores[i] = new Scores();
                    
                    //Score each particle on rectangularity and aspect ratio
                    scores[i].rectangularity = scoreRectangularity(report);
                    scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
                    scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);
                    
                    //Check if the particle is a horizontal target, if not, check if its vertical
                    if (scoreCompare(scores[i], false)) {
                        System.out.println("particle: " + i + "is a Horizontal Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                        horizontalTargets[horizontalTargetCount++] = i;
                    }
                    else if (scoreCompare(scores[i], true)) {
                        System.out.println("particle: " + i + "is a Vertical Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                        verticalTargets[verticalTargetCount++] = i;
                    }
                    else {
                        System.out.println("particle: " + i + "is not a Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                    }
                }
                
                //Zero out scores and set verticalIndex to first target in case there are no horizontal targets
                target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
                target.distance = -1; //Negative distance to mark as invalid
                target.verticalIndex = verticalTargets[0];
                
                for (int i = 0; i < verticalTargetCount; i++) {
                    ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);
                    for (int j = 0; j < horizontalTargetCount; j++) {
                        ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
                        double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;
                        
                        //Measure equivalent rectangle sides for use in score calculation
                        horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
                        vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                        horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                        
                        //Determine if the horizontal target is in the expected location to the left of the vertical target
                        leftScore = ratioToScore(1.2 * (verticalReport.boundingRectLeft - horizontalReport.center_mass_x) / horizWidth);
                        //Determine if the horizontal target is in the expected location to the right of the vertical target
                        rightScore = ratioToScore(1.2 * (horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth) / horizWidth);
                        //Determine if the width of the tape on the two targets appears to be the same
                        tapeWidthScore = ratioToScore(vertWidth / horizHeight);
                        //Determine if the vertical location of the horizontal target appears to be correct
                        verticalScore = ratioToScore(1 - (verticalReport.boundingRectTop - horizontalReport.center_mass_y) / (4 * horizHeight));
                        total = leftScore > rightScore ? leftScore:rightScore;
                        total += tapeWidthScore + verticalScore;
                        
                        //If the target is the best detected so far store the information about it
                        if (total > target.totalScore) {
                            target.horizontalIndex = horizontalTargets[j];
                            target.verticalIndex = verticalTargets[i];
                            target.totalScore = total;
                            target.leftScore = leftScore;
                            target.rightScore = rightScore;
                            target.tapeWidthScore = tapeWidthScore;
                            target.verticalScore = verticalScore;
                        }
                    }
                    
                    target.Hot = hotOrNot(target);
                }
                
                if (verticalTargetCount > 0) {
                    //Info about target is contained in the target structure
                    //To get measurement info such as size or locations, use the
                    //horiz or vert index to get the particle report
                    ParticleAnalysisReport distanceReport = filteredImage.getParticleAnalysisReport(target.verticalIndex);
                    double distance = computeDistance(filteredImage, distanceReport, target.verticalIndex);
                    //Save the distance
                    target.distance = distance;
                    if (target.Hot) {
                        System.out.println("Hot target located");
                        System.out.println("Distance: " + distance);
                    }
                    else {
                        System.out.println("No hot target present");
                        System.out.println("Distance: " + distance);
                    }
                }
            }
            
            filteredImage.free();
            thresholdImage.free();
            image.free();
            
            return target;
            
        } catch (NIVisionException e) {
            e.printStackTrace();
        } catch (AxisCameraException e) {
            e.printStackTrace();
        }
        
        return target;
    }
    
    //=== Scoring functions
    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
     * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
     * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area = x*y
     * and particle perimeter = 2x+2y
     * 
     * @param image The image containing the particle to score, needed to perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the width, height and particle number
     * @param particleNumber Particle index offset
     * @param vertical Vertical or not
     * @return
     * @throws NIVisionException 
     */
    private double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException {
        double rectLong, rectShort, aspectRatio, idealAspectRatio;
        
        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = vertical ? (4.0/32) : (23.5/4); //Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
        
        //Divide width by height to measure aspect ratio
        if (report.boundingRectWidth > report.boundingRectHeight) {
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore((rectLong / rectShort) / idealAspectRatio);
        }
        else {
            aspectRatio = ratioToScore((rectShort / rectLong) / idealAspectRatio);
        }
        return aspectRatio;
    }
    
    /**
     * Compares scores to defined limits and returns true if the particle appears to be a target
     * 
     * @param scores
     * @param vertical
     * @return true if the particle meets all limits, false otherwise
     */
    private boolean scoreCompare(Scores scores, boolean vertical) {
        boolean isTarget = true;
        
        isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
        if (vertical) {
            isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
        }
        else {
            isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
        }
        
        return isTarget;
    }
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
     * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box
     * 
     * @param report
     * @return The rectangularity score (0-100)
     */
    private double scoreRectangularity(ParticleAnalysisReport report) {
        if (report.boundingRectWidth * report.boundingRectHeight != 0) {
            return 100 * report.particleArea / (report.boundingRectWidth * report.boundingRectHeight);
        }
        else {
            return 0;
        }
    }
    
    /**
     * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
     * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside range 0-2
     * @param ratio
     * @return 
     */
    private double ratioToScore(double ratio) {
        return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
    }
    
    /**
     * Takes in a report on a target and compares the scores to the defined score limits to evaluate
     * if the target is a hot target or not
     * 
     * @param target
     * @return True if the target is hot, false otherwise
     */
    private boolean hotOrNot(TargetReport target) {
        boolean isHot = true;
        
        isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
        isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
        isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
        
        return isHot;
    }
    
    /**
     * Computes the estimated distance to the target using the height of the particle in the image
     * 
     * @param image The image to usefor measuring the particle estimated rectangle
     * @param report The particle analysis report for the particle
     * @param particleNumber
     * @return The estimated distance to the target in inches
     * @throws NIVisionException 
     */
    private double computeDistance (BinaryImage image, ParticleAnalysisReport report, int particleNumber) throws NIVisionException {
        double rectLong, height;
        int targetHeight;
        
        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        //using the smaller of the estimated rectangle long side and the bounding rectangle height results in better 
        //performance on skewed rectangles
        height = Math.min(report.boundingRectHeight, rectLong);
        targetHeight = 32;
        
        return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
    }
}
