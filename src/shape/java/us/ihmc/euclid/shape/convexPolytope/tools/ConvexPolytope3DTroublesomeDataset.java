package us.ihmc.euclid.shape.convexPolytope.tools;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.function.Function;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class ConvexPolytope3DTroublesomeDataset
{
   protected ConvexPolytope3D convexPolytope3D;
   protected final Point3D troublesomePoint = new Point3D();

   public Point3D getTroublesomePoint()
   {
      return troublesomePoint;
   }

   public double getConstructionEpsilon()
   {
      return convexPolytope3D.getConstructionEpsilon();
   }

   public ConvexPolytope3D getConvexPolytope3D()
   {
      return convexPolytope3D;
   }

   public static String generateDatasetAsString(List<? extends Tuple3DReadOnly> pointsBeforeIssue, Tuple3DReadOnly troublesomePoint, double constructionEpsilon)
   {
      String stringFormat = EuclidCoreIOTools.getStringFormat(23, 20);
      String baseName = ConvexPolytope3DTroublesomeDataset.class.getSimpleName();
      String newClassName = baseName + "_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
      String result = "public class " + newClassName + " extends " + baseName + " {\n";
      result += "public " + newClassName + "() {\n";

      result += "List<Point3D> pointsBeforeIssue = new ArrayList<>();\n";
      String prefix = "pointsBeforeIssue.add(new Point3D";
      String suffix = ");";
      String separator = suffix + "\n" + prefix;
      Function<Tuple3DReadOnly, String> elementToStringFunction = v -> EuclidCoreIOTools.getTuple3DString(stringFormat, v);

      result += EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, pointsBeforeIssue, elementToStringFunction) + "\n";
      result += "troublesomePoint.set" + elementToStringFunction.apply(troublesomePoint) + ";\n";
      result += "convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(pointsBeforeIssue), " + constructionEpsilon + ");}}";

      return result;
   }

   public static String generateDatasetAsString(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      return generateDatasetAsString(convexPolytope3D, new Point3D());
   }

   public static String generateDatasetAsString(ConvexPolytope3DReadOnly convexPolytope3D, Tuple3DReadOnly troublesomePoint)
   {
      String stringFormat = EuclidCoreIOTools.getStringFormat(23, 20);
      String baseName = ConvexPolytope3DTroublesomeDataset.class.getSimpleName();
      String newClassName = baseName + "_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
      String result = "public class " + newClassName + " extends " + baseName + " {\n";
      result += "public " + newClassName + "() {\n";

      double constructionEpsilon = convexPolytope3D.getConstructionEpsilon();
      List<? extends Vertex3DReadOnly> vertices = convexPolytope3D.getVertices();

      result += "double constructionEpsilon = " + constructionEpsilon + ";\n";

      Function<Tuple3DReadOnly, String> newVertex3D = v -> "Vertex3D v" + vertices.indexOf(v) + " = new Vertex3D"
            + EuclidCoreIOTools.getTuple3DString(stringFormat, v) + ";\n";
      result += EuclidCoreIOTools.getCollectionString("", convexPolytope3D.getVertices(), newVertex3D);

      Function<HalfEdge3DReadOnly, String> newHalfEdge3DFunction = e -> "new HalfEdge3D(v" + vertices.indexOf(e.getOrigin()) + ", v"
            + vertices.indexOf(e.getDestination()) + ")";
      Function<Face3DReadOnly, String> newFace3DFunction = f -> EuclidCoreIOTools.getCollectionString("new Face3D(Arrays.asList(", "), new Vector3D"
            + EuclidCoreIOTools.getTuple3DString(stringFormat, f.getNormal()) + ", constructionEpsilon)", ", ", f.getEdges(), newHalfEdge3DFunction);

      result += "List<Face3D> faces = new ArrayList<>();\n";
      String prefix = "faces.add(";
      String suffix = ");";
      String separator = suffix + "\n" + prefix;
      result += EuclidCoreIOTools.getCollectionString(prefix, suffix + "\n", separator, convexPolytope3D.getFaces(), newFace3DFunction);
      result += "convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);\n";
      result += "troublesomePoint.set" + EuclidCoreIOTools.getTuple3DString(stringFormat, troublesomePoint) + ";}}";

      return result;
   }
}
