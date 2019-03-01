package us.ihmc.euclid.shape.convexPolytope.tools;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class ConvexPolytope3DTroublesomeDataset
{
   protected final List<Point3D> pointsBeforeIssue = new ArrayList<>();
   protected final Point3D troublesomePoint = new Point3D();
   protected double constructionEpsilon;

   public ConvexPolytope3DTroublesomeDataset()
   {
   }

   public List<Point3D> getPointsBeforeIssue()
   {
      return pointsBeforeIssue;
   }

   public Vertex3DSupplier getPointsBeforeIssueAsSupplier()
   {
      return Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue);
   }

   public Point3D getTroublesomePoint()
   {
      return troublesomePoint;
   }

   public double getConstructionEpsilon()
   {
      return constructionEpsilon;
   }

   public static String generateDatasetAsString(ConvexPolytope3DReadOnly convexPolytope3DBeforeIssue, Tuple3DReadOnly troublesomePoint)
   {
      return generateDatasetAsString(convexPolytope3DBeforeIssue.getVertices(), troublesomePoint, convexPolytope3DBeforeIssue.getConstructionEpsilon());
   }

   public static String generateDatasetAsString(List<? extends Tuple3DReadOnly> pointsBeforeIssue, Tuple3DReadOnly troublesomePoint, double constructionEpsilon)
   {
      String stringFormat = EuclidCoreIOTools.getStringFormat(23, 20);
      String baseName = ConvexPolytope3DTroublesomeDataset.class.getSimpleName();
      String newClassName = baseName + "_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
      String result = "public class " + newClassName + " extends " + baseName + " {\n";
      result += "public " + newClassName + "() {\n";

      String prefix = "pointsBeforeIssue.add(new Point3D";
      String suffix = ");";
      String separator = suffix + "\n" + prefix;
      Function<Tuple3DReadOnly, String> elementToStringFunction = v -> EuclidCoreIOTools.getTuple3DString(stringFormat, v);

      result += EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, pointsBeforeIssue, elementToStringFunction) + "\n";
      result += "troublesomePoint.set" + elementToStringFunction.apply(troublesomePoint) + ";\n";
      result += "constructionEpsilon = " + constructionEpsilon + ";}}";

      return result;
   }

   public static void main(String[] args)
   {
      System.out.println(generateDatasetAsString(EuclidShapeRandomTools.nextConvexPolytope3D(new Random()), EuclidCoreRandomTools.nextPoint3D(new Random())));
   }
}
