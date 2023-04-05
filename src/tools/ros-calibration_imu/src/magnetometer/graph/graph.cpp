#include "graph.h"

#include "common/geometry/ellipsoid.h"

using namespace magnetometer;

// CONSTRUCTORS
graph::graph(std::shared_ptr<magnetometer::data_interface>& data_interface, std::shared_ptr<magnetometer::calibrator>& calibrator)
{
    // Store components.
    graph::m_data_interface = data_interface;
    graph::m_calibrator = calibrator;

    // Initialize scatter instance.
    graph::m_graph = new QtDataVisualization::Q3DScatter();
    graph::m_graph->axisX()->setTitle("x");
    graph::m_graph->axisX()->setTitleVisible(true);
    graph::m_graph->axisX()->setSegmentCount(10);
    graph::m_graph->axisY()->setTitle("z");
    graph::m_graph->axisY()->setTitleVisible(true);
    graph::m_graph->axisY()->setSegmentCount(10);
    graph::m_graph->axisZ()->setTitle("y");
    graph::m_graph->axisZ()->setTitleVisible(true);
    graph::m_graph->axisZ()->setSegmentCount(10);
    graph::m_graph->setAspectRatio(1.0);
    graph::m_graph->setHorizontalAspectRatio(1.0);
    graph::m_graph->setShadowQuality(QtDataVisualization::QAbstract3DGraph::ShadowQuality::ShadowQualityNone);

    // Initialize uncalibrated series.
    graph::m_series_uncalibrated = new QtDataVisualization::QScatter3DSeries();
    graph::m_series_uncalibrated->setBaseColor(QColor(Qt::GlobalColor::blue));
    graph::m_series_uncalibrated->setItemSize(0.075);
    graph::m_graph->addSeries(graph::m_series_uncalibrated);

    // Initialize uncalibrated newest series.
    graph::m_series_uncalibrated_new = new QtDataVisualization::QScatter3DSeries();
    graph::m_series_uncalibrated_new->setBaseColor(QColor(Qt::GlobalColor::red));
    graph::m_series_uncalibrated_new->setItemSize(0.2);
    graph::m_graph->addSeries(graph::m_series_uncalibrated_new);

    // Initialize fit series.
    graph::m_series_fit = new QtDataVisualization::QScatter3DSeries();
    graph::m_series_fit->setBaseColor(QColor(Qt::GlobalColor::cyan));
    graph::m_series_fit->setItemSize(0.075);
    graph::m_graph->addSeries(graph::m_series_fit);

    // Initialize calibrated series.
    graph::m_series_calibrated = new QtDataVisualization::QScatter3DSeries();
    graph::m_series_calibrated->setBaseColor(QColor(Qt::GlobalColor::darkGreen));
    graph::m_series_calibrated->setItemSize(0.075);
    graph::m_graph->addSeries(graph::m_series_calibrated);

    // Initialize truth series.
    graph::m_series_truth = new QtDataVisualization::QScatter3DSeries();
    graph::m_series_truth->setBaseColor(QColor(Qt::GlobalColor::green));
    graph::m_series_truth->setItemSize(0.075);
    graph::m_graph->addSeries(graph::m_series_truth);

    // Initialize flags.
    graph::f_indicate_new_point = true;

    // Connect slots.
    connect(graph::m_data_interface.get(), &magnetometer::data_interface::data_updated, this, &magnetometer::graph::update_uncalibrated_plot);
    connect(graph::m_calibrator.get(), &magnetometer::calibrator::calibration_completed, this, &magnetometer::graph::update_calibration_plots);
}
graph::~graph()
{
    // Clean up pointers.
    delete graph::m_graph;
}

// INITIALIZATION
QWidget* graph::get_widget()
{
    return QWidget::createWindowContainer(graph::m_graph);
}

// PLOT VISIBILITY
void graph::uncalibrated_visible(bool visible)
{
    graph::m_series_uncalibrated->setVisible(visible);
    graph::m_series_uncalibrated_new->setVisible(visible);
}
void graph::fit_visible(bool visible)
{
    graph::m_series_fit->setVisible(visible);
}
void graph::calibrated_visible(bool visible)
{
    graph::m_series_calibrated->setVisible(visible);
}
void graph::truth_visible(bool visible)
{
    graph::m_series_truth->setVisible(visible);
}
void graph::indicate_new_point(bool enabled)
{
    bool update_series = (enabled != graph::f_indicate_new_point);
    graph::f_indicate_new_point = enabled;

    if(update_series)
    {
        graph::update_uncalibrated_plot();
    }
}

// PLOT UPDATE METHODS
void graph::update_uncalibrated_plot()
{
    // Add all uncalibrated data points to the series.

    // Get total number of points.
    uint32_t n_points = graph::m_data_interface->n_points();

    // Create new data arrays.
    QtDataVisualization::QScatterDataArray* uncalibrated_points = new QtDataVisualization::QScatterDataArray();
    QtDataVisualization::QScatterDataArray* new_point = new QtDataVisualization::QScatterDataArray();

    // Reserve space in the uncalibrated points.
    uncalibrated_points->reserve(n_points);

    // Populate uncalibrated points array.
    for(uint32_t i = 0; i < n_points; ++i)
    {
        QVector3D point;
        graph::m_data_interface->get_point(i, point);
        // Scale point.
        point *= graph::m_field_scale;
        // Add point.
        uncalibrated_points->append(point);
    }

    // Set up newest point if enabled.
    if(graph::f_indicate_new_point && n_points > 0)
    {
        new_point->append(uncalibrated_points->takeLast());
    }

    // Add to series.
    graph::m_series_uncalibrated->dataProxy()->resetArray(uncalibrated_points);
    graph::m_series_uncalibrated_new->dataProxy()->resetArray(new_point);

    // Update axis scales.
    graph::autoscale();
}
void graph::update_calibration_plots(bool calibration_success)
{
    // Check if calibration was successful.
    if(calibration_success)
    {
        // Draw fit.
        Eigen::Vector3d fit_center, fit_radius, fit_rotation;
        graph::m_calibrator->get_fit(fit_center, fit_radius, fit_rotation);
        // Scale center and radius.
        fit_center *= graph::m_field_scale;
        fit_radius *= graph::m_field_scale;
        common::ellipsoid fit;
        fit.set_center(fit_center);
        fit.set_radius(fit_radius);
        fit.set_rotation(fit_rotation);
        std::vector<Eigen::Vector3d> fit_points;
        fit.draw(fit_points);
        graph::m_series_fit->dataProxy()->resetArray(graph::draw(fit_points));

        // Draw calibrated points.
        // Get calibration.
        Eigen::Matrix4d calibration;
        graph::m_calibrator->get_calibration(calibration);
        // Iterate through uncalibrated points to apply calibration and draw into series.
        QtDataVisualization::QScatterDataArray* calibrated_points = new QtDataVisualization::QScatterDataArray();
        uint32_t n_points = graph::m_data_interface->n_points();
        Eigen::Vector3d p;
        Eigen::Vector4d p_u, p_c;
        p_u(3) = 1;
        for(uint32_t i = 0; i < n_points; ++i)
        {
            // Grab point.
            graph::m_data_interface->get_point(i, p);
            p_u.block(0, 0, 3, 1) = p;
            // Calibrate point.
            p_c.noalias() = calibration * p_u;
            // Scale calibrated point.
            p_c *= graph::m_field_scale;
            // Add calibrated point to series.
            calibrated_points->append(QVector3D(p_c(0), p_c(2), p_c(1)));
        }
        // Add points to series.
        graph::m_series_calibrated->dataProxy()->resetArray(calibrated_points);
    }

    // Update axis scales.
    graph::autoscale();
}
void graph::update_truth_plot(double true_field_strength)
{
    // Draw truth.

    // Create truth ellipsoid.
    common::ellipsoid truth;
    Eigen::Vector3d center, radius, rotation;
    center.setZero();
    radius.fill(true_field_strength * graph::m_field_scale);
    rotation.setZero();
    truth.set_center(center);
    truth.set_radius(radius);
    truth.set_rotation(rotation);

    // Draw ellipsoid.
    std::vector<Eigen::Vector3d> truth_points;
    truth.draw(truth_points);
    graph::m_series_truth->dataProxy()->resetArray(graph::draw(truth_points));
}

// PLOT METHODS
QtDataVisualization::QScatterDataArray *graph::draw(const std::vector<Eigen::Vector3d>& source)
{
    QtDataVisualization::QScatterDataArray* output = new QtDataVisualization::QScatterDataArray();

    for(auto point = source.cbegin(); point != source.cend(); ++point)
    {
        output->append(QVector3D((*point)(0), (*point)(2), (*point)(1)));
    }

    return output;
}
void graph::autoscale()
{
    // Determine min/max x/y/z values of all points in all series.
    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();

    // Update range through each series.
    graph::update_range(graph::m_series_uncalibrated, min, max);
    graph::update_range(graph::m_series_uncalibrated_new, min, max);
    graph::update_range(graph::m_series_fit, min, max);
    graph::update_range(graph::m_series_calibrated, min, max);
    graph::update_range(graph::m_series_truth, min, max);

    // Check if a range was calculated.
    if(std::isinf(min) || std::isinf(max))
    {
        // No range calculated, abort.
        return;
    }

    // Update axis ranges.
    float margin = 5;
    float range_min = min - margin;
    float range_max = max + margin;
    // Round to nearest factor of 10.
    range_min = static_cast<int32_t>((range_min + (float)(range_min >= 0) * (10 - 1)) / 10) * 10.0;
    range_max = static_cast<int32_t>((range_max + (float)(range_max >= 0) * (10 - 1)) / 10) * 10.0;
    // Set ranges.
    graph::m_graph->axisX()->setRange(range_min, range_max);
    graph::m_graph->axisY()->setRange(range_min, range_max);
    graph::m_graph->axisZ()->setRange(range_min, range_max);
}
void graph::update_range(QtDataVisualization::QScatter3DSeries* series, float &min, float &max)
{
    for(auto point = series->dataProxy()->array()->cbegin(); point != series->dataProxy()->array()->cend(); ++point)
    {
        // Update x range.
        if(point->x() < min)
        {
            min = point->x();
        }
        else if(point->x() > max)
        {
            max = point->x();
        }

        // Update y range.
        if(point->y() < min)
        {
            min = point->y();
        }
        else if(point->y() > max)
        {
            max = point->y();
        }

        // Update z range.
        if(point->z() < min)
        {
            min = point->z();
        }
        else if(point->z() > max)
        {
            max = point->z();
        }
    }
}
