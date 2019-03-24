package us.ihmc.euclid.shape.tools;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

// https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
public class EuclidEllipsoid3DTools
{
   public static final int DEFAULT_ROOT_FINDING_ITERATIONS = 150;
   public static final double DEFAULT_EPSILON = 1.0e-10;

   private static double findRoot(double r0, double r1, double z0, double z1, double z2, double g, int maxIterations, double epsilon)
   {
      double n0 = r0 * z0;
      double n1 = r1 * z1;
      double s0 = z2 - 1.0;
      double s1 = g < 0.0 ? 0.0 : (Math.sqrt(EuclidCoreTools.normSquared(n0, n1, z2)) - 1.0);
      double s = 0.0;

      for (int i = 0; i < maxIterations; i++)
      {
         s = (s0 + s1) / 2.0;
         if (s == s0 || s == s1)
            break;
         double ratio0 = n0 / (s + r0);
         double ratio1 = n1 / (s + r1);
         double ratio2 = z2 / (s + 1.0);
         g = EuclidCoreTools.normSquared(ratio0, ratio1, ratio2) - 1.0;
         if (g > epsilon)
            s0 = s;
         else if (g < -epsilon)
            s1 = s;
         else
            break;
      }

      return s;
   }

   private static double findRoot(double r0, double z0, double z1, double g, int maxIterations, double epsilon)
   {
      double n0 = r0 * z0;
      double s0 = z1 - 1.0;
      double s1 = g < 0.0 ? 0.0 : (Math.sqrt(EuclidCoreTools.normSquared(n0, z1)) - 1.0);
      double s = 0.0;

      for (int i = 0; i < maxIterations; i++)
      {
         s = (s0 + s1) / 2.0;
         if (s == s0 || s == s1)
            break;
         double ratio0 = n0 / (s + r0);
         double ratio1 = z1 / (s + 1.0);
         g = EuclidCoreTools.normSquared(ratio0, ratio1) - 1.0;
         if (g > epsilon)
            s0 = s;
         else if (g < -epsilon)
            s1 = s;
         else
            break;
      }

      return s;
   }

   private enum CoordinateOrder implements Tuple3DUpdater
   {
      XYZ((x0, x1, x2, tuple) -> tuple.set(x0, x1, x2), tuple -> tuple.getX(), tuple -> tuple.getY(), tuple -> tuple.getZ()),
      XZY((x0, x1, x2, tuple) -> tuple.set(x0, x2, x1), tuple -> tuple.getX(), tuple -> tuple.getZ(), tuple -> tuple.getY()),
      YXZ((x0, x1, x2, tuple) -> tuple.set(x1, x0, x2), tuple -> tuple.getY(), tuple -> tuple.getX(), tuple -> tuple.getZ()),
      YZX((x0, x1, x2, tuple) -> tuple.set(x2, x0, x1), tuple -> tuple.getY(), tuple -> tuple.getZ(), tuple -> tuple.getX()),
      ZXY((x0, x1, x2, tuple) -> tuple.set(x1, x2, x0), tuple -> tuple.getZ(), tuple -> tuple.getX(), tuple -> tuple.getY()),
      ZYX((x0, x1, x2, tuple) -> tuple.set(x2, x1, x0), tuple -> tuple.getZ(), tuple -> tuple.getY(), tuple -> tuple.getX());

      private final Tuple3DUpdater tuple3DUpdater;
      private final CoordinateReader coordinateReader0;
      private final CoordinateReader coordinateReader1;
      private final CoordinateReader coordinateReader2;

      private CoordinateOrder(Tuple3DUpdater tuple3DUpdater, CoordinateReader coordinateReader0, CoordinateReader coordinateReader1,
                              CoordinateReader coordinateReader2)
      {
         this.tuple3DUpdater = tuple3DUpdater;
         this.coordinateReader0 = coordinateReader0;
         this.coordinateReader1 = coordinateReader1;
         this.coordinateReader2 = coordinateReader2;
      }

      @Override
      public void updateTuple3D(double x0, double x1, double x2, Tuple3DBasics tupleToUpdate)
      {
         tuple3DUpdater.updateTuple3D(x0, x1, x2, tupleToUpdate);
      }

      public double getCoordinate0(Tuple3DReadOnly tuple)
      {
         return coordinateReader0.read(tuple);
      }

      public double getCoordinate1(Tuple3DReadOnly tuple)
      {
         return coordinateReader1.read(tuple);
      }

      public double getCoordinate2(Tuple3DReadOnly tuple)
      {
         return coordinateReader2.read(tuple);
      }

      static CoordinateOrder descendingOrder(Tuple3DReadOnly tuple)
      {
         if (tuple.getX() >= tuple.getY())
         {
            if (tuple.getY() >= tuple.getZ())
            {
               return CoordinateOrder.XYZ;
            }
            else if (tuple.getX() >= tuple.getZ())
            {
               return CoordinateOrder.XZY;
            }
            else
            {
               return CoordinateOrder.ZXY;
            }
         }
         else
         {
            if (tuple.getX() >= tuple.getZ())
            {
               return CoordinateOrder.YXZ;
            }
            else if (tuple.getY() >= tuple.getZ())
            {
               return CoordinateOrder.YZX;
            }
            else
            {
               return CoordinateOrder.ZYX;
            }
         }
      }
   };

   private interface Tuple3DUpdater
   {
      void updateTuple3D(double x0, double x1, double x2, Tuple3DBasics tupleToUpdate);
   }

   private interface CoordinateReader
   {
      double read(Tuple3DReadOnly tuple);
   }

   private static double square(double value)
   {
      return value * value;
   }

   public static double distancePoint3DEllipsoid3D(Vector3DReadOnly radii, Point3DReadOnly query)
   {
      return distancePoint3DEllipsoid3D(radii, query, null);
   }

   public static double distancePoint3DEllipsoid3D(Vector3DReadOnly radii, Point3DReadOnly query, Point3DBasics closestPointToPack)
   {
      return distancePoint3DEllipsoid3D(radii, query, DEFAULT_ROOT_FINDING_ITERATIONS, DEFAULT_EPSILON, closestPointToPack);
   }

   public static double distancePoint3DEllipsoid3D(Vector3DReadOnly radii, Point3DReadOnly query, int maxIterations, double epsilon,
                                                   Point3DBasics closestPointToPack)
   {
      // Enforcing: e0 > e1 > e2
      CoordinateOrder order = CoordinateOrder.descendingOrder(radii);
      double e0 = order.getCoordinate0(radii);
      double e1 = order.getCoordinate1(radii);
      double e2 = order.getCoordinate2(radii);
      double y0 = order.getCoordinate0(query);
      double y1 = order.getCoordinate1(query);
      double y2 = order.getCoordinate2(query);
      boolean negate0 = y0 < 0.0;
      boolean negate1 = y1 < 0.0;
      boolean negate2 = y2 < 0.0;

      if (negate0)
         y0 = -y0;
      if (negate1)
         y1 = -y1;
      if (negate2)
         y2 = -y2;

      double x0 = Double.NaN;
      double x1 = Double.NaN;
      double x2 = Double.NaN;
      double distance = Double.NaN;

      if (y2 > epsilon)
      {
         if (y1 > epsilon)
         {
            if (y0 > epsilon)
            {
               double z0 = y0 / e0;
               double z1 = y1 / e1;
               double z2 = y2 / e2;
               double g = EuclidCoreTools.normSquared(z0, z1, z2) - 1.0;

               if (!EuclidCoreTools.isZero(g, epsilon))
               {
                  double r0 = square(e0 / e2);
                  double r1 = square(e1 / e2);
                  double sbar = findRoot(r0, r1, z0, z1, z2, g, maxIterations, epsilon);
                  x0 = r0 * y0 / (sbar + r0);
                  x1 = r1 * y1 / (sbar + r1);
                  x2 = y2 / (sbar + 1.0);
                  distance = EuclidGeometryTools.distanceBetweenPoint3Ds(x0, x1, x2, y0, y1, y2);
                  if (g < 0.0)
                     distance = -distance;
               }
               else
               {
                  x0 = y0;
                  x1 = y1;
                  x2 = y2;
                  distance = 0.0;
               }
            }
            else // y0 == 0.0
            {
               distance = distancePoint2DEllipse2D(e1, e2, y1, y2, maxIterations, epsilon, closestPointToPack);
               if (closestPointToPack != null)
               {
                  x0 = 0.0;
                  x1 = closestPointToPack.getX();
                  x2 = closestPointToPack.getY();
               }
            }
         }
         else // y1 == 0.0
         {
            if (y0 > epsilon)
            {
               distance = distancePoint2DEllipse2D(e0, e2, y0, y2, maxIterations, epsilon, closestPointToPack);
               if (closestPointToPack != null)
               {
                  x0 = closestPointToPack.getX();
                  x1 = 0.0;
                  x2 = closestPointToPack.getY();
               }
            }
            else // y0 == 0.0
            {
               x0 = 0.0;
               x1 = 0.0;
               x2 = e2;
               distance = y2 - e2;
            }
         }
      }
      else // y2 == 0.0
      {
         double denom0 = square(e0) - square(e2);
         double denom1 = square(e1) - square(e2);
         double numer0 = e0 * y0;
         double numer1 = e1 * y1;
         boolean computed = false;

         if (numer0 < denom0 && numer1 < denom1)
         {
            double xde0 = numer0 / denom0;
            double xde1 = numer1 / denom1;
            double discr = 1.0 - square(xde0) - square(xde1);
            if (discr > epsilon)
            {
               x0 = e0 * xde0;
               x1 = e1 * xde1;
               x2 = e2 * Math.sqrt(discr);
               distance = -EuclidGeometryTools.distanceBetweenPoint3Ds(x0, x1, x2, y0, y1, 0.0);
               computed = true;
            }
         }

         if (!computed)
         {
            distance = distancePoint2DEllipse2D(e0, e1, y0, y1, maxIterations, epsilon, closestPointToPack);
            if (closestPointToPack != null)
            {
               x0 = closestPointToPack.getX();
               x1 = closestPointToPack.getY();
               x2 = 0.0;
            }
         }
      }

      if (negate0)
         x0 = -x0;
      if (negate1)
         x1 = -x1;
      if (negate2)
         x2 = -x2;

      if (closestPointToPack != null)
         order.updateTuple3D(x0, x1, x2, closestPointToPack);

      return distance;
   }

   private static double distancePoint2DEllipse2D(double e0, double e1, double y0, double y1, int maxIterations, double epsilon,
                                                  Point3DBasics closestPointToPack)
   {
      double distance;
      double x0, x1;

      if (y1 > epsilon)
      {
         if (y0 > epsilon)
         {
            double z0 = y0 / e0;
            double z1 = y1 / e1;
            double g = EuclidCoreTools.normSquared(z0, z1) - 1.0;

            if (!EuclidCoreTools.isZero(g, epsilon))
            {
               double r0 = square(e0 / e1);
               double sbar = findRoot(r0, z0, z1, g, maxIterations, epsilon);
               x0 = r0 * y0 / (sbar + r0);
               x1 = y1 / (sbar + 1.0);
               distance = EuclidGeometryTools.distanceBetweenPoint2Ds(x0, x1, y0, y1);
               if (g < 0.0)
                  distance = -distance;
            }
            else
            {
               x0 = y0;
               x1 = y1;
               distance = 0.0;
            }
         }
         else // y0 == 0.0
         {
            x0 = 0.0;
            x1 = e1;
            distance = y1 - e1;
         }
      }
      else // y1 == 0.0
      {
         double numer0 = e0 * y0;
         double denom0 = square(e0) - square(e1);
         if (numer0 < denom0)
         {
            double xde0 = numer0 / denom0;
            x0 = e0 * xde0;
            x1 = e1 * Math.sqrt(1.0 - square(xde0));
            distance = Math.sqrt(square(x0 - y0) + square(x1));
         }
         else
         {
            x0 = e0;
            x1 = 0.0;
            distance = y0 - e0;
         }
      }
      if (closestPointToPack != null)
         closestPointToPack.set(x0, x1, Double.NaN);
      return distance;
   }
}
