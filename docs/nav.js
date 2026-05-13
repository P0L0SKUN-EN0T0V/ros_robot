(function() {
    const current = location.pathname.split('/').pop() || 'guide.html';

    const sections = [
        { type: 'link', href: 'guide.html', text: 'Обзор проекта' },
        { type: 'sep' },
        { type: 'title', text: 'Пошаговый гайд' },
        { type: 'link', href: 'step-0-what-is-ros2.html', text: '0. Что такое ROS2' },
        { type: 'link', href: 'step-1-tools-gazebo-rviz.html', text: '1. Gazebo и RViz' },
        { type: 'link', href: 'step-2-sensors.html', text: '2. Одометрия и лидар' },
        { type: 'link', href: 'step-3-mapping.html', text: '3. Строим карту' },
        { type: 'link', href: 'step-4-astar.html', text: '4. Алгоритм A*' },
        { type: 'link', href: 'step-5-pure-pursuit.html', text: '5. Pure Pursuit' },
        { type: 'link', href: 'step-6-assembly.html', text: '6. Собираем проект' },
        { type: 'sep' },
        { type: 'title', text: 'Алгоритмы' },
        { type: 'link', href: 'algo-1-astar.html', text: '1. A*' },
        { type: 'link', href: 'algo-2-dijkstra.html', text: '2. Dijkstra' },
        { type: 'link', href: 'algo-3-rrt.html', text: '3. RRT' },
        { type: 'link', href: 'algo-4-rrt-star.html', text: '4. RRT*' },
        { type: 'link', href: 'algo-5-theta-star.html', text: '5. Theta*' },
        { type: 'link', href: 'algo-6-dstar-lite.html', text: '6. D* Lite' },
        { type: 'link', href: 'algo-7-jps.html', text: '7. Jump Point Search' },
        { type: 'link', href: 'algo-8-lpa-star.html', text: '8. LPA*' },
        { type: 'link', href: 'algo-9-hybrid-astar.html', text: '9. Hybrid A*' },
        { type: 'link', href: 'algo-10-prm.html', text: '10. PRM' },
    ];

    const nav = document.getElementById('sidebar');
    if (!nav) return;

    const h3 = document.createElement('h3');
    h3.textContent = 'ROS2 Navigation';
    nav.appendChild(h3);

    sections.forEach(function(item) {
        if (item.type === 'sep') {
            var div = document.createElement('div');
            div.className = 'sep';
            nav.appendChild(div);
        } else if (item.type === 'title') {
            var div = document.createElement('div');
            div.className = 'section-title';
            div.textContent = item.text;
            nav.appendChild(div);
        } else if (item.type === 'link') {
            var a = document.createElement('a');
            a.href = item.href;
            a.textContent = item.text;
            if (item.href === current) a.className = 'active';
            nav.appendChild(a);
        }
    });
})();
