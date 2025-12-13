import type {ReactNode} from 'react';

export default function RecentUpdates(): ReactNode {
  const updates = [
    {
      date: '2025-12-10',
      title: 'Homepage Redesign v2 Complete',
      description: 'Completed homepage redesign with minimal aesthetic and improved navigation.',
    },
    {
      date: '2025-12-09',
      title: 'Module 4 Implementation',
      description: 'Finalized Module 4 VLA & Humanoid Robotics with complete documentation.',
    },
  ];

  return (
    <section className="mt-16">
      <h2 className="text-3xl font-bold text-gray-900 mb-8">Recent Updates</h2>
      <div className="space-y-4">
        {updates.map((update, idx) => (
          <div key={idx} className="border-l-4 border-green-600 pl-4 py-2">
            <div className="text-sm text-gray-500 font-medium">{update.date}</div>
            <h3 className="text-lg font-bold text-gray-900 mt-1">{update.title}</h3>
            <p className="text-gray-600 mt-2">{update.description}</p>
          </div>
        ))}
      </div>
    </section>
  );
}
